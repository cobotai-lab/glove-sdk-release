"""Axis-aligned adaptive layer for hand forward kinematics."""
import numpy as np
import torch
from torch.nn import Module

from .handlayer import HandLayer
from .utils.geometry import matrix_to_euler_angles, euler_angles_to_matrix, rotation_to_axis_angle


class AxisAdaptiveLayer(torch.nn.Module):

    def __init__(self, side: str = "right"):
        super(AxisAdaptiveLayer, self).__init__()
        self.joints_mapping = [5, 6, 7, 9, 10, 11, 17, 18, 19, 13, 14, 15, 1, 2, 3]
        self.parent_joints_mappings = [0, 5, 6, 0, 9, 10, 0, 17, 18, 0, 13, 14, 0, 1, 2]
        self.side = side
        if side == "right":
            up_axis_base = np.vstack((np.array([[0, 1, 0]]).repeat(13, axis=0), np.array([[1, 1, 1]]).repeat(3, axis=0)))
        elif side == "left":
            up_axis_base = np.vstack((np.array([[0, 1, 0]]).repeat(13, axis=0), np.array([[-1, 1, 1]]).repeat(3, axis=0)))
        self.register_buffer("up_axis_base", torch.from_numpy(up_axis_base).float().unsqueeze(0))

    def forward(self, hand_joints, transf):
        bs = transf.shape[0]
        b_axis = hand_joints[:, self.parent_joints_mappings] - hand_joints[:, self.joints_mapping]
        b_axis = (transf[:, 1:, :3, :3].transpose(2, 3) @ b_axis.unsqueeze(-1)).squeeze(-1)
        if self.side == "right":
            b_axis_init = torch.tensor([1, 0, 0]).float().unsqueeze(0).unsqueeze(0).repeat(bs, 1, 1).to(b_axis.device)
        elif self.side == "left":
            b_axis_init = torch.tensor([-1, 0, 0]).float().unsqueeze(0).unsqueeze(0).repeat(bs, 1, 1).to(b_axis.device)
        b_axis = torch.cat((b_axis_init, b_axis), dim=1)

        l_axis = torch.cross(b_axis, self.up_axis_base.expand(bs, 16, 3))
        u_axis = torch.cross(l_axis, b_axis)

        return (
            b_axis / torch.norm(b_axis, dim=2, keepdim=True),
            u_axis / torch.norm(u_axis, dim=2, keepdim=True),
            l_axis / torch.norm(l_axis, dim=2, keepdim=True),
        )


class AxisLayerFK(Module):

    def __init__(self, side: str = "right", hand_assets_root: str = "assets/hand_model"):
        super(AxisLayerFK, self).__init__()
        self.transf_parent_mapping = [0, 0, 1, 2, 0, 4, 5, 0, 7, 8, 0, 10, 11, 0, 13, 14]

        tmpl_pose = torch.zeros(1, 48)
        tmpl_shape = torch.zeros(1, 10)
        tmpl = HandLayer(side=side, hand_assets_root=hand_assets_root)(tmpl_pose, tmpl_shape)
        tmpl_joints = tmpl.joints
        tmpl_transf_abs = tmpl.transforms_abs

        tmpl_b_axis, tmpl_u_axis, tmpl_l_axis = AxisAdaptiveLayer(side=side)(tmpl_joints, tmpl_transf_abs)
        tmpl_R_p_a = torch.cat((tmpl_b_axis.unsqueeze(-1), tmpl_u_axis.unsqueeze(-1), tmpl_l_axis.unsqueeze(-1)), dim=3)
        zero_tsl = torch.zeros(1, 16, 3, 1)
        zero_pad = torch.tensor([[[[0, 0, 0, 1]]]]).repeat(*zero_tsl.shape[0:2], 1, 1)
        _tmpl_T_p_a = torch.cat((tmpl_R_p_a, zero_tsl), dim=3)
        tmpl_T_p_a = torch.cat((_tmpl_T_p_a, zero_pad), dim=2)
        tmpl_T_g_a = torch.matmul(tmpl_transf_abs, tmpl_T_p_a)
        self.register_buffer("TMPL_T_p_a", tmpl_T_p_a.float())
        self.register_buffer("TMPL_R_p_a", tmpl_R_p_a.float())
        self.register_buffer("TMPL_T_g_a", tmpl_T_g_a.float())

    def forward(self, transf):
        T_g_p = transf
        R_g_p = T_g_p[:, :, :3, :3]
        R_g_a = torch.matmul(R_g_p, self.TMPL_R_p_a)
        T_g_a = torch.cat((R_g_a, T_g_p[:, :, :3, 3:]), dim=3)
        zero_pad = torch.tensor([[[[0, 0, 0, 1]]]]).repeat(*T_g_a.shape[0:2], 1, 1).to(T_g_a.device)
        T_g_a = torch.cat((T_g_a, zero_pad), dim=2)

        Ta_par_chd = torch.matmul(T_g_a[:, self.transf_parent_mapping, ...].transpose(2, 3), T_g_a)
        Ra_par_chd = Ta_par_chd[:, :, :3, :3]

        Ra_par_tmplchd = torch.matmul(self.TMPL_R_p_a[:, self.transf_parent_mapping, ...].transpose(2, 3),
                                      self.TMPL_R_p_a)
        Ra_chd_tmplchd = torch.matmul(Ra_par_chd.transpose(2, 3), Ra_par_tmplchd)
        Ra_tmplchd_chd = Ra_chd_tmplchd.transpose(2, 3)

        ee_a_tmplchd_chd = matrix_to_euler_angles(Ra_tmplchd_chd, convention="XYZ")
        return T_g_a, Ra_tmplchd_chd, ee_a_tmplchd_chd

    def compose(self, angles):
        ee_tmplchd_chd = angles
        Ra_tmplchd_chd = euler_angles_to_matrix(ee_tmplchd_chd, convention="XYZ")

        Ra_par_tmplchd = torch.matmul(self.TMPL_R_p_a[:, self.transf_parent_mapping, ...].transpose(2, 3),
                                      self.TMPL_R_p_a)
        Ra_par_chd = torch.matmul(Ra_par_tmplchd, Ra_tmplchd_chd)

        lev1_idxs = [1, 4, 7, 10, 13]
        lev2_idxs = [2, 5, 8, 11, 14]
        lev3_idxs = [3, 6, 9, 12, 15]

        all_rot_chains = [Ra_par_chd[:, 0:1]]
        lev1_rots = Ra_par_chd[:, [idx for idx in lev1_idxs]]
        lev2_rots = Ra_par_chd[:, [idx for idx in lev2_idxs]]
        lev3_rots = Ra_par_chd[:, [idx for idx in lev3_idxs]]

        lev1_rot_chains = torch.matmul(Ra_par_chd[:, 0:1].repeat(1, 5, 1, 1), lev1_rots)
        all_rot_chains.append(lev1_rot_chains)
        lev2_rot_chains = torch.matmul(lev1_rot_chains, lev2_rots)
        all_rot_chains.append(lev2_rot_chains)
        lev3_rot_chains = torch.matmul(lev2_rot_chains, lev3_rots)
        all_rot_chains.append(lev3_rot_chains)
        reorder_idxs = [0, 1, 6, 11, 2, 7, 12, 3, 8, 13, 4, 9, 14, 5, 10, 15]
        R_g_a = torch.cat(all_rot_chains, 1)[:, reorder_idxs]
        R_g_p = torch.matmul(R_g_a, self.TMPL_R_p_a.transpose(2, 3))

        Rp_par_chd = torch.matmul(R_g_p[:, self.transf_parent_mapping].transpose(2, 3), R_g_p)
        aa_p_par_chd = rotation_to_axis_angle(Rp_par_chd)
        return aa_p_par_chd
