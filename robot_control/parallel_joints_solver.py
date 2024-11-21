from math import asin, sqrt
import numpy as np

def global_init():
    import torch
    torch.set_num_threads(1)

# @njit
def single_radian_rotation_matrix(angle_radian: np.float64) -> np.ndarray:
    rotation_matrix = np.zeros((3, 3))
    rotation_matrix[0, 0] = np.cos(angle_radian)
    rotation_matrix[0, 2] = np.sin(angle_radian)
    rotation_matrix[1, 1] = 1
    rotation_matrix[2, 0] = -np.sin(angle_radian)
    rotation_matrix[2, 2] = np.cos(angle_radian)

    return rotation_matrix


# @njit
def radian_to_rotation_matrix(roll: np.float64, pitch: np.float64) -> np.ndarray:
    roll_matrix = np.zeros((3, 3))
    roll_matrix[0, 0] = 1
    roll_matrix[1, 1] = np.cos(roll)
    roll_matrix[1, 2] = -np.sin(roll)
    roll_matrix[2, 1] = np.sin(roll)
    roll_matrix[2, 2] = np.cos(roll)

    pitch_matrix = np.zeros((3, 3))
    pitch_matrix[0, 0] = np.cos(pitch)
    pitch_matrix[0, 2] = np.sin(pitch)
    pitch_matrix[1, 1] = 1
    pitch_matrix[2, 0] = -np.sin(pitch)
    pitch_matrix[2, 2] = np.cos(pitch)

    return np.dot(pitch_matrix, roll_matrix)


#################### working range ####################
# motor: -60~30 deg # -1.0472~0.5236 rad # forwad input
# ankle(roll): -25~25 deg, -0.43663~0.4363 rad
# ankle(pitch): -60~30 deg, -1.0472~0.5236 rad
#######################################################


class ParallelAnkle:
    def __init__(self, type):
        # params and configuration
        if type == "left":
            self.isLeft = True
            self.initial_ra1 = np.array([0, 25, 260])  # 初始化位置参数、连杆长度、踝关节旋转角度和力矩、旋量
            self.initial_ra2 = np.array([0, -25, 205])
            self.initial_rb1 = np.array([-53.95, 25, 270.68])
            self.initial_rb2 = np.array([-53.95, -25, 215.68])
            self.initial_rc1 = np.array([-53.95, 25, 10.68])
            self.initial_rc2 = np.array([-53.95, -25, 10.68])
            self.length_bar1 = 55
            self.length_bar2 = 55
            self.length_rod1 = 260
            self.length_rod2 = 205
            self.initial_offset = 11.2
        elif type == "right":
            self.isLeft = False
            self.initial_ra1 = np.array([0, 25, 205])
            self.initial_ra2 = np.array([0, -25, 260])
            self.initial_rb1 = np.array([-53.95, 25, 215.68])
            self.initial_rb2 = np.array([-53.95, -25, 270.68])
            self.initial_rc1 = np.array([-53.95, 25, 10.68])
            self.initial_rc2 = np.array([-53.95, -25, 10.68])
            self.length_bar1 = 55
            self.length_bar2 = 55
            self.length_rod1 = 205
            self.length_rod2 = 260
            self.initial_offset = 11.2

    def left_fk_nnfit(self, joint_position_l, joint_position_r):
        b_a = np.array(
            [
                4.1611978856994787,
                4.28741375744369,
                -3.2329482724835192,
                3.2295728584301391,
                3.365863656368993,
                -3.4259786092261972,
                0.303635264829151,
                1.0357905491413777,
                1.5231961933374518,
                1.0498917678443165,
                -0.58362691644491116,
                1.7484159415008411,
                2.1113609630711081,
                1.0747125999655283,
                3.5508508673796273,
                -5.6997289141674745,
                -0.36377691922178579,
                3.8197442841339533,
                2.9072686042308167,
                4.4845259887083584,
                3.0252680728927888,
                -4.4380907596964221,
                3.4572240063471118,
                2.2951870935425682,
                2.1190189047177417,
                1.9493717780635309,
                3.61377945932344,
                -1.9231356367313859,
                1.1284908864145931,
                -1.4455242366115082,
                0.2762312104770232,
                -2.2592374860789031,
                -0.9919894744965122,
                -2.1345868264164469,
                -4.1318070917760625,
                1.3700001439411873,
                1.9476927947853835,
                -0.47108463478070545,
                5.9982905586314432,
                -4.4738324363998334,
            ]
        )

        c_a = np.array(
            [
                -0.029794316585132038,
                0.01074737121138849,
                -0.058219617245258738,
                1.320440270982024,
                -0.812445434526058,
                0.85385039357651027,
                -0.019969054245140574,
                0.0079475300319278418,
                -0.0056757536194365658,
                0.0021853594840013433,
                -0.022823271077105488,
                -0.011099589162747449,
                -0.14441661402650102,
                0.0055976007721327955,
                1.9685634137657466,
                -0.4000332665044134,
                -0.02085288266606752,
                0.00960177457386128,
                0.63899221309889553,
                -0.67205744539590417,
                -1.8269641153308076,
                -2.6893496524019556,
                0.36562432032970271,
                -0.24121790972856549,
                0.261320393324544,
                0.064926999361325163,
                -1.0585982747652551,
                0.16535903667133425,
                0.21883920552105279,
                -0.10727643303014929,
                0.29247156508798017,
                0.012678685702453714,
                0.4383921027264408,
                -0.069479006307976557,
                1.2393962580759492,
                0.040974566668789285,
                0.00022042510584849409,
                -0.00010354397897001635,
                0.47070224310908043,
                0.098821292850216055,
            ]
        )

        a = np.array(
            [
                -5.9760158537189847,
                -4.1249691705803508,
                3.2066607341023725,
                -3.5720327546704826,
                -2.4546116983536455,
                1.8517622251157368,
                4.404202269458251,
                -0.87384290494696482,
                -0.85240856674642762,
                0.065454587653003743,
                0.058225544026714843,
                1.1254504621582928,
                1.3112153092199585,
                -0.86300243857635028,
                2.7755539240786171,
                -5.4439882066443595,
                -2.500743964487075,
                4.6395292436252324,
                3.3321784177238882,
                6.1120838188927777,
            ]
        )

        b = np.zeros(20)
        xp1 = np.array([-0.1639267685074135, 0.61135792158094282])
        ankle_position = np.zeros(2)

        d = (joint_position_l - -1.2256) * 1.0158007811508 + -1.0
        d1 = (joint_position_r - -1.2261) * 1.01606397138764 + -1.0

        for k in range(20):
            b[k] = 2.0 / (np.exp(-2.0 * (a[k] + (b_a[k] * d + b_a[k + 20] * d1))) + 1.0) - 1.0

        for k in range(2):
            d = 0.0
            for i in range(20):
                d += c_a[k + (i << 1)] * b[i]
            ankle_position[k] = ((xp1[k] + d) - -1.0) / (-1.01860676223578 * float(k) + 2.29184332958999) + (
                -0.61086999999999991 * float(k) + -0.43633
            )

        # (ankle_position_roll, ankle_position_pitch)
        return ankle_position[0], ankle_position[1]

    def right_fk_nnfit(self, joint_position_l, joint_position_r):
        b_a = np.array(
            [
                5.8015802628833963,
                5.5325401941629586,
                -3.1505991231686941,
                -1.6915759867734201,
                -0.66507918822980661,
                -0.67238047065067741,
                -1.6663152753691004,
                -1.8304042900954436,
                -0.864526210917737,
                0.538361899407137,
                -0.77791921527080321,
                1.2916881014140098,
                -3.3447817087599736,
                1.7969729102051304,
                -3.4977469290567171,
                -3.613223353210175,
                3.5411609071599148,
                -2.7773046317734376,
                3.8705604825857307,
                -2.0056279141110647,
                1.2754353671617,
                1.140460934393515,
                0.13051461255442884,
                -1.4773611913026237,
                -2.0003509287100094,
                -1.9455181279387057,
                3.6493474874193628,
                0.66617788014938006,
                0.48943252027258788,
                0.46615879246825032,
                1.5161007739518868,
                -0.54082471251291553,
                -4.000751452938653,
                0.45733541304994479,
                -1.5828175080949114,
                -1.6496482114185858,
                2.04459900329226,
                -3.8725970521498554,
                -3.2058205465663394,
                -3.6934113164268672,
            ]
        )

        c_a = np.array(
            [
                -0.52055268433455837,
                -0.013064426993101631,
                0.66391725663750567,
                0.019889562027824745,
                0.8069642301199792,
                0.13739004104960595,
                -0.025791917069473709,
                -0.18778517593056288,
                -0.41929815057738729,
                1.0149687569993062,
                0.46353307812404443,
                -1.1364794900870865,
                0.015683098719050922,
                0.0077122779793951284,
                0.73268754746115372,
                0.13786361779282369,
                -7.2012919550334438,
                -0.91596425278923554,
                -1.9496074237077998,
                0.92922246644371409,
                0.45437945232945731,
                0.13841680375736659,
                3.1938362166761407,
                -0.30901667502786556,
                0.0016089429365942402,
                -0.00033021148932226209,
                -0.32109445292991867,
                0.1154325220587684,
                1.0674143383026604,
                -0.37680019193500852,
                -0.89626271414466752,
                0.31228596323969338,
                -0.15727994506526946,
                0.077378763634186221,
                0.55143815685033315,
                -0.42124328495441182,
                1.0468702194073807,
                -1.1538345882850085,
                -0.548632191810631,
                0.54129620778662479,
            ]
        )

        a = np.array(
            [
                -6.1083164296326329,
                -5.8301729310806731,
                3.4229839770536832,
                3.0223018402152344,
                1.4640724379919925,
                1.4469175835095069,
                0.9314251848390066,
                1.117096416081601,
                0.92261500183104561,
                -0.037504437646810745,
                -0.14710242985866709,
                0.97160335969268163,
                -1.8245064590468378,
                0.88123673337471642,
                -2.9476764717238542,
                -3.0327050448250619,
                4.5034889371670168,
                -6.4125243423026648,
                4.3845371782546385,
                -5.7719103289008293,
            ]
        )

        b = np.zeros(20)
        xp1 = np.array([1.06957653882809, 2.0739829450581366])
        ankle_position = np.zeros(2)

        d = (joint_position_l - -1.2258) * 1.01620852598953 + -1.0
        d1 = (joint_position_r - -1.2259) * 1.01565634251994 + -1.0

        for k in range(20):
            b[k] = 2.0 / (np.exp(-2.0 * (a[k] + (b_a[k] * d + b_a[k + 20] * d1))) + 1.0) - 1.0

        for k in range(2):
            d = 0.0
            for i in range(20):
                d += c_a[k + (i << 1)] * b[i]
            ankle_position[k] = ((xp1[k] + d) - -1.0) / (-1.01860676223578 * float(k) + 2.29184332958999) + (
                -0.61086999999999991 * float(k) + -0.43633
            )

        # (ankle_position_roll, ankle_position_pitch)
        return ankle_position[0], ankle_position[1]

    def forward(
        self,
        joint_position_up_deg,
        joint_position_lower_deg,
        joint_velocity_up_deg=0,
        joint_velocity_lower_deg=0,
        joint_torque_up=0,
        joint_torque_lower=0,
    ):
        # ankle position
        if self.isLeft:
            joint_position_l = np.deg2rad(joint_position_up_deg)
            joint_position_r = np.deg2rad(joint_position_lower_deg)
            joint_velocity_l = np.deg2rad(joint_velocity_up_deg)
            joint_velocity_r = np.deg2rad(joint_velocity_lower_deg)
            joint_torque_l = joint_torque_up
            joint_torque_r = joint_torque_lower
            ankle_position_roll, ankle_position_pitch = self.left_fk_nnfit(joint_position_l, joint_position_r)
        else:
            joint_position_l = np.deg2rad(joint_position_lower_deg)
            joint_position_r = np.deg2rad(joint_position_up_deg)
            joint_velocity_l = np.deg2rad(joint_velocity_lower_deg)
            joint_velocity_r = np.deg2rad(joint_velocity_up_deg)
            joint_torque_l = joint_torque_lower
            joint_torque_r = joint_torque_up
            ankle_position_roll, ankle_position_pitch = self.right_fk_nnfit(joint_position_l, joint_position_r)

        rotation_matrix = radian_to_rotation_matrix(ankle_position_roll, ankle_position_pitch)
        target_rc1 = np.dot(rotation_matrix, self.initial_rc1)
        target_rc2 = np.dot(rotation_matrix, self.initial_rc2)
        target_ra1 = self.initial_ra1
        target_ra2 = self.initial_ra2

        # 根据角度得到旋转矩阵
        single_rotation_matrix_l = single_radian_rotation_matrix(joint_position_l)
        single_rotation_matrix_r = single_radian_rotation_matrix(joint_position_r)

        # 踝关节目标位置参数
        target_rb1 = target_ra1 + np.dot(single_rotation_matrix_l, (self.initial_rb1 - self.initial_ra1))
        target_rb2 = target_ra2 + np.dot(single_rotation_matrix_r, (self.initial_rb2 - self.initial_ra2))

        # bar和rod的向量表示
        r_bar1 = target_rb1 - target_ra1
        r_bar2 = target_rb2 - target_ra2
        r_rod1 = target_rc1 - target_rb1
        r_rod2 = target_rc2 - target_rb2

        # 旋量中的方向向量
        s11 = np.array([0, 1, 0])
        s21 = np.array([0, 1, 0])

        # Jx
        Jx = np.array(
            [
                r_rod1.tolist() + (np.cross(target_rc1, r_rod1)).tolist(),
                r_rod2.tolist() + (np.cross(target_rc2, r_rod2)).tolist(),
            ]
        )

        # J_theta
        J_theta = np.array([[np.dot(s11, np.cross(r_bar1, r_rod1)), 0], [0, np.dot(s21, np.cross(r_bar2, r_rod2))]])

        # G_matrix
        G_matrix = np.array(
            [[0, 0, 0, np.cos(ankle_position_pitch), 0, -np.sin(ankle_position_pitch)], [0, 0, 0, 0, 1, 0]]
        ).T

        # ankle velocity
        joint_velocity = np.array([joint_velocity_l, joint_velocity_r])
        ankle_velocity_roll, ankle_velocity_pitch = np.linalg.inv(
            np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix))
        ).dot(joint_velocity)

        # ankle torque
        joint_torque = np.array([joint_torque_l, joint_torque_r])
        ankle_torque_roll, ankle_torque_pitch = np.linalg.inv(
            np.linalg.inv(np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix))).T
        ).dot(joint_torque)

        return (
            np.rad2deg(ankle_position_pitch),
            np.rad2deg(ankle_position_roll),
            np.rad2deg(ankle_velocity_pitch),
            np.rad2deg(ankle_velocity_roll),
            (ankle_torque_pitch),
            (ankle_torque_roll),
        )  # numpy.floa64

    def inverse(
        self,
        ankle_position_pitch_deg,
        ankle_position_roll_deg,
        ankle_velocity_pitch_deg=0,
        ankle_velocity_roll_deg=0,
        ankle_torque_pitch=0,
        ankle_torque_roll=0,
    ):
        ankle_position_pitch = np.deg2rad(ankle_position_pitch_deg)
        ankle_position_roll = np.deg2rad(ankle_position_roll_deg)
        ankle_velocity_pitch = np.deg2rad(ankle_velocity_pitch_deg)
        ankle_velocity_roll = np.deg2rad(ankle_velocity_roll_deg)
        ankle_torque_pitch = ankle_torque_pitch
        ankle_torque_roll = ankle_torque_roll

        rotation_matrix = radian_to_rotation_matrix(ankle_position_roll, ankle_position_pitch)

        # 旋转后的关节点
        target_rc1 = np.dot(rotation_matrix, self.initial_rc1)
        target_rc2 = np.dot(rotation_matrix, self.initial_rc2)
        target_ra1 = self.initial_ra1
        target_ra2 = self.initial_ra2

        # 得到计算公式的元素
        interm_a1 = target_rc1 - target_ra1
        a1 = interm_a1[0]
        interm_a2 = target_rc2 - target_ra2
        a2 = interm_a2[0]
        interm_b1 = target_ra1 - target_rc1
        b1 = interm_b1[2]
        interm_b2 = target_ra2 - target_rc2
        b2 = interm_b2[2]

        # 计算二阶范数
        norm_1 = np.linalg.norm(target_rc1 - target_ra1, ord=2)
        norm_2 = np.linalg.norm(target_rc2 - target_ra2, ord=2)

        c1 = (self.length_rod1**2 - self.length_bar1**2 - norm_1**2) / (2 * self.length_bar1)
        c2 = (self.length_rod2**2 - self.length_bar2**2 - norm_2**2) / (2 * self.length_bar2)

        # joint position
        joint_position_l = asin(
            (b1 * c1 + sqrt(b1**2 * c1**2 - (a1**2 + b1**2) * (c1**2 - a1**2))) / (a1**2 + b1**2)
        ) - np.deg2rad(self.initial_offset)
        joint_position_r = asin(
            (b2 * c2 + sqrt(b2**2 * c2**2 - (a2**2 + b2**2) * (c2**2 - a2**2))) / (a2**2 + b2**2)
        ) - np.deg2rad(self.initial_offset)

        single_rotation_matrix_l = single_radian_rotation_matrix(joint_position_l)
        single_rotation_matrix_r = single_radian_rotation_matrix(joint_position_r)

        # 踝关节目标位置参数
        target_rb1 = target_ra1 + np.dot(single_rotation_matrix_l, (self.initial_rb1 - self.initial_ra1))
        target_rb2 = target_ra2 + np.dot(single_rotation_matrix_r, (self.initial_rb2 - self.initial_ra2))

        # bar和rod的向量表示
        r_bar1 = target_rb1 - target_ra1
        r_bar2 = target_rb2 - target_ra2
        r_rod1 = target_rc1 - target_rb1
        r_rod2 = target_rc2 - target_rb2

        # 旋量中的方向向量
        s11 = np.array([0, 1, 0])
        s21 = np.array([0, 1, 0])

        # 雅可比矩阵的组成部分
        # Jx
        Jx = np.array(
            [
                r_rod1.tolist() + (np.cross(target_rc1, r_rod1)).tolist(),
                r_rod2.tolist() + (np.cross(target_rc2, r_rod2)).tolist(),
            ]
        )

        # J_theta
        J_theta = np.array([[np.dot(s11, np.cross(r_bar1, r_rod1)), 0], [0, np.dot(s21, np.cross(r_bar2, r_rod2))]])

        # G_matrix
        G_matrix = np.array(
            [[0, 0, 0, np.cos(ankle_position_pitch), 0, -np.sin(ankle_position_pitch)], [0, 0, 0, 0, 1, 0]]
        ).T

        # joint velocity
        ankle_velocity = np.array([ankle_velocity_roll, ankle_velocity_pitch])
        joint_velocity_l, joint_velocity_r = np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix)).dot(ankle_velocity)

        # joint torque
        ankle_torque = np.array([ankle_torque_roll, ankle_torque_pitch])
        joint_torque_l, joint_torque_r = np.linalg.inv(np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix))).T.dot(
            ankle_torque
        )

        if self.isLeft:
            return (
                np.rad2deg(joint_position_l),
                np.rad2deg(joint_position_r),
                np.rad2deg(joint_velocity_l),
                np.rad2deg(joint_velocity_r),
                (joint_torque_l),
                (joint_torque_r),
            )
        else:
            return (
                np.rad2deg(joint_position_r),
                np.rad2deg(joint_position_l),
                np.rad2deg(joint_velocity_r),
                np.rad2deg(joint_velocity_l),
                (joint_torque_r),
                (joint_torque_l),
            )


########### 注意 ###########
####### 统一单位为角度 #######
##### 电机顺序：先左后右 #####
## 姿态顺序：先pitch后roll ##
###########################

class ParallelHead:
    def __init__(self):
        # params and configuration
        self.initial_ra1 = np.array([0, 9.5, 44])
        self.initial_ra2 = np.array([0, -9.5, 44])
        self.initial_rb1 = np.array([-15, 9.5, 44])
        self.initial_rb2 = np.array([-15, -9.5, 44])
        self.initial_rc1 = np.array([-15, 9.5, 0])
        self.initial_rc2 = np.array([-15, -9.5, 0])
        self.length_bar1 = 15
        self.length_bar2 = 15
        self.length_rod1 = 44
        self.length_rod2 = 44
        self.initial_offset = 0.0

    def fk_nnfit(self, joint_position_1, joint_position_2):
        b_a = np.array(
            [
                1.8697050897768714,
                3.7864772428487563,
                -4.6492250336758127,
                1.6841323782857136,
                0.32740766363234114,
                -3.29072051411786,
                -1.7728576509294045,
                0.40682090837901969,
                1.1720697242428078,
                0.96643983361688146,
                -0.87395354276592641,
                1.1039102413786415,
                0.78279882868712858,
                0.19308504133869406,
                2.2284423212148852,
                -3.0570141899810821,
                -0.54222340371243272,
                3.6274770889455734,
                -1.7682554776870532,
                4.5397202573334416,
                3.8462945443182575,
                -4.1489378418383742,
                2.7179548265074991,
                4.5424983792375766,
                2.0575239402843324,
                1.3756057212538322,
                1.1562239181632421,
                -2.0201447828945107,
                0.0597003606513327,
                -1.7515510619660961,
                0.24568896228416973,
                -2.104749783480214,
                -0.086037955875044467,
                -2.0468006518287551,
                -1.7476021471922762,
                1.301115129863688,
                2.18323852282883,
                -2.2771542412715817,
                5.2366315269337207,
                -4.8762506354508135,
            ]
        )

        c_a = np.array(
            [
                0.11129238753250602,
                -0.0075582658405282587,
                1.1890827049253863,
                0.047606121377929474,
                -0.56600969534800338,
                -0.031249131358117015,
                0.016337736514226178,
                -0.0010633353049691614,
                0.29828021628742118,
                -0.030884569387188469,
                2.4949208252224579,
                0.2456333057839509,
                -0.74313329834907316,
                0.32230574282767349,
                1.1691037836118066,
                -0.20721752296911233,
                -0.93390300017787919,
                -0.22980821084451844,
                0.48947398215768129,
                -0.3498139044308114,
                0.25777639301294208,
                -1.5707178010685026,
                0.33021514231754179,
                -0.19204239494716857,
                3.0183169118181787,
                1.759138295620237,
                -0.78871527554834464,
                0.096351436074942148,
                1.6043267306959139,
                -1.0618208502491764,
                2.8241723827160929,
                0.97225551850086633,
                -0.75606948074435676,
                0.17170014769002712,
                0.15662916594429216,
                0.57777140411348182,
                0.280254852126239,
                -0.042303082075480357,
                0.3744316477411555,
                0.47492605898192036,
            ]
        )

        a = np.array(
            [
                -5.016900378781747,
                -4.859068343933318,
                4.08109798021488,
                -4.11173788330037,
                -1.5555944793197813,
                3.7712795722418289,
                1.3985617833923696,
                -1.3998745441760658,
                0.8499660909540242,
                -0.34831341594551085,
                0.90033157042193257,
                0.56626723249982558,
                0.685100639363494,
                -1.4437325397676997,
                2.6879886510162034,
                -3.6799380893547906,
                -1.5952987309441458,
                3.5743511319219232,
                5.1255980010435858,
                5.6338001100438682,
            ]
        )

        b = np.zeros(20)
        xp1 = np.array([-0.96645995634095694, 0.77778212009644032])
        neck_position = np.zeros(2)

        d = (joint_position_1 - -1.3516) * 0.753068755177348 - 1.0
        d1 = (joint_position_2 - -1.3523) * 0.752587017873942 - 1.0

        for k in range(20):
            b[k] = 2.0 / (np.exp(-2.0 * (a[k] + (b_a[k] * d + b_a[k + 20] * d1))) + 1.0) - 1.0

        for k in range(2):
            d = 0.0
            for i in range(20):
                d += c_a[k + (i << 1)] * b[i]
            neck_position[k] = ((xp1[k] + d) - -1.0) / (-0.76393318623633011 * float(k) + 1.90985485103132) + (
                    -0.34906000000000004 * float(k) + -0.5236
            )

        # (neck_position_roll, neck_position_pitch)
        return neck_position[0], neck_position[1]

    def forward(
            self,
            joint_position_l_deg,
            joint_position_r_deg,
            joint_velocity_l_deg=0,
            joint_velocity_r_deg=0,
            joint_torque_l=0,
            joint_torque_r=0,
    ):
        joint_position_l = np.deg2rad(joint_position_l_deg)
        joint_position_r = -np.deg2rad(joint_position_r_deg)
        joint_velocity_l = np.deg2rad(joint_velocity_l_deg)
        joint_velocity_r = -np.deg2rad(joint_velocity_r_deg)
        neck_position_roll, neck_position_pitch = self.fk_nnfit(joint_position_l, joint_position_r)

        rotation_matrix = radian_to_rotation_matrix(neck_position_roll, neck_position_pitch)
        target_rc1 = np.dot(rotation_matrix, self.initial_rc1)
        target_rc2 = np.dot(rotation_matrix, self.initial_rc2)
        target_ra1 = self.initial_ra1
        target_ra2 = self.initial_ra2

        # 根据角度得到旋转矩阵
        single_rotation_matrix_l = single_radian_rotation_matrix(joint_position_l)
        single_rotation_matrix_r = single_radian_rotation_matrix(joint_position_r)

        # 踝关节目标位置参数
        target_rb1 = target_ra1 + np.dot(single_rotation_matrix_l, (self.initial_rb1 - self.initial_ra1))
        target_rb2 = target_ra2 + np.dot(single_rotation_matrix_r, (self.initial_rb2 - self.initial_ra2))

        # bar和rod的向量表示
        r_bar1 = target_rb1 - target_ra1
        r_bar2 = target_rb2 - target_ra2
        r_rod1 = target_rc1 - target_rb1
        r_rod2 = target_rc2 - target_rb2

        # 旋量中的方向向量
        s11 = np.array([0, 1, 0])
        s21 = np.array([0, 1, 0])

        # Jx
        Jx = np.array(
            [
                r_rod1.tolist() + (np.cross(target_rc1, r_rod1)).tolist(),
                r_rod2.tolist() + (np.cross(target_rc2, r_rod2)).tolist(),
            ]
        )

        # J_theta
        J_theta = np.array([[np.dot(s11, np.cross(r_bar1, r_rod1)), 0], [0, np.dot(s21, np.cross(r_bar2, r_rod2))]])

        # G_matrix
        G_matrix = np.array(
            [[0, 0, 0, np.cos(neck_position_pitch), 0, -np.sin(neck_position_pitch)], [0, 0, 0, 0, 1, 0]]
        ).T

        # neck velocity
        joint_velocity = np.array([joint_velocity_l, joint_velocity_r])
        neck_velocity_roll, neck_velocity_pitch = np.linalg.inv(
            np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix))
        ).dot(joint_velocity)

        # neck torque
        joint_torque = np.array([joint_torque_l, joint_torque_r])
        neck_torque_roll, neck_torque_pitch = np.linalg.inv(
            np.linalg.inv(np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix))).T
        ).dot(joint_torque)

        return (
            -np.rad2deg(neck_position_pitch),
            -np.rad2deg(neck_position_roll),
            np.rad2deg(neck_velocity_pitch),
            np.rad2deg(neck_velocity_roll),
            (neck_torque_pitch),
            (neck_torque_roll),
        )  # numpy.floa64

    def inverse(
            self,
            neck_position_pitch_deg,
            neck_position_roll_deg,
            neck_velocity_pitch_deg=0,
            neck_velocity_roll_deg=0,
            neck_torque_pitch=0,
            neck_torque_roll=0,
    ):
        neck_position_pitch = np.deg2rad(neck_position_pitch_deg)
        neck_position_roll = np.deg2rad(neck_position_roll_deg)
        neck_velocity_pitch = np.deg2rad(neck_velocity_pitch_deg)
        neck_velocity_roll = np.deg2rad(neck_velocity_roll_deg)

        rotation_matrix = radian_to_rotation_matrix(neck_position_roll, neck_position_pitch)

        # 旋转后的关节点
        target_rc1 = np.dot(rotation_matrix, self.initial_rc1)
        target_rc2 = np.dot(rotation_matrix, self.initial_rc2)
        target_ra1 = self.initial_ra1
        target_ra2 = self.initial_ra2

        # 得到计算公式的元素
        interm_a1 = target_rc1 - target_ra1
        a1 = interm_a1[0]
        interm_a2 = target_rc2 - target_ra2
        a2 = interm_a2[0]
        interm_b1 = target_ra1 - target_rc1
        b1 = interm_b1[2]
        interm_b2 = target_ra2 - target_rc2
        b2 = interm_b2[2]

        # 计算二阶范数
        norm_1 = np.linalg.norm(target_rc1 - target_ra1, ord=2)
        norm_2 = np.linalg.norm(target_rc2 - target_ra2, ord=2)

        c1 = (self.length_rod1 ** 2 - self.length_bar1 ** 2 - norm_1 ** 2) / (2 * self.length_bar1)
        c2 = (self.length_rod2 ** 2 - self.length_bar2 ** 2 - norm_2 ** 2) / (2 * self.length_bar2)

        # motor position
        joint_position_l = asin(
            (b1 * c1 + sqrt(b1 ** 2 * c1 ** 2 - (a1 ** 2 + b1 ** 2) * (c1 ** 2 - a1 ** 2))) / (a1 ** 2 + b1 ** 2)
        ) - np.deg2rad(self.initial_offset)
        joint_position_r = asin(
            (b2 * c2 + sqrt(b2 ** 2 * c2 ** 2 - (a2 ** 2 + b2 ** 2) * (c2 ** 2 - a2 ** 2))) / (a2 ** 2 + b2 ** 2)
        ) - np.deg2rad(self.initial_offset)

        single_rotation_matrix_l = single_radian_rotation_matrix(joint_position_l)
        single_rotation_matrix_r = single_radian_rotation_matrix(joint_position_r)

        # 踝关节目标位置参数
        target_rb1 = target_ra1 + np.dot(single_rotation_matrix_l, (self.initial_rb1 - self.initial_ra1))
        target_rb2 = target_ra2 + np.dot(single_rotation_matrix_r, (self.initial_rb2 - self.initial_ra2))

        # bar和rod的向量表示
        r_bar1 = target_rb1 - target_ra1
        r_bar2 = target_rb2 - target_ra2
        r_rod1 = target_rc1 - target_rb1
        r_rod2 = target_rc2 - target_rb2

        # 旋量中的方向向量
        s11 = np.array([0, 1, 0])
        s21 = np.array([0, 1, 0])

        # 雅可比矩阵的组成部分
        # Jx
        Jx = np.array(
            [
                r_rod1.tolist() + (np.cross(target_rc1, r_rod1)).tolist(),
                r_rod2.tolist() + (np.cross(target_rc2, r_rod2)).tolist(),
            ]
        )

        # J_theta
        J_theta = np.array([[np.dot(s11, np.cross(r_bar1, r_rod1)), 0], [0, np.dot(s21, np.cross(r_bar2, r_rod2))]])

        # G_matrix
        G_matrix = np.array(
            [[0, 0, 0, np.cos(neck_position_pitch), 0, -np.sin(neck_position_pitch)], [0, 0, 0, 0, 1, 0]]
        ).T

        # motor velocity
        neck_velocity = np.array([neck_velocity_roll, neck_velocity_pitch])
        joint_velocity_l, joint_velocity_r = np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix)).dot(neck_velocity)

        # motor torque
        neck_torque = np.array([neck_torque_roll, neck_torque_pitch])
        joint_torque_l, joint_torque_r = np.linalg.inv(np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix))).T.dot(
            neck_torque
        )

        return (
            -1 * np.rad2deg(joint_position_l),
            -1 * -np.rad2deg(joint_position_r),
            np.rad2deg(joint_velocity_l),
            -np.rad2deg(joint_velocity_r),
            (joint_torque_l),
            (joint_torque_r),
        )


##################### 注意 ######################
############# 输入输出已按照角度制统一 #############
# 坐标系已根据电机默认正方向与urdf定义的关节正方向匹配 #
# 正解的输入顺序为由上至下（即电机IP读取顺序）#########
# 逆解的输出顺序为先roll后pitch（即urdf定义顺序）#####
################################################


class ParallelWrist:
    def __init__(self, type):
        # params and configuration
        if type == "left":
            self.isLeft = True
        elif type == "right":
            self.isLeft = False
        self.initial_ra1 = np.array([-6.25, 8.48, 43.55])
        self.initial_ra2 = np.array([-6.25, -8.48, 43.55])
        self.initial_rb1 = np.array([-21.25, 8.48, 43.55])
        self.initial_rb2 = np.array([-21.25, -8.48, 43.55])
        self.initial_rc1 = np.array([-15, 8.48, 0])
        self.initial_rc2 = np.array([-15, -8.48, 0])
        self.length_bar1 = 15
        self.length_bar2 = 15
        self.length_rod1 = 44
        self.length_rod2 = 44
        self.initial_offset = 0.0

    # left and right hand share the same network
    def fk_nnfit(self, joint_position_1, joint_position_2):
        b_a = np.array(
            [
                6.6083239733578063,
                3.9418421836336042,
                -2.4579755519699353,
                1.108162573868255,
                3.8232989517854667,
                -6.7010279006829805,
                -3.90939104979405,
                0.69440382071476536,
                1.6276165341395585,
                0.67750057218553084,
                -1.9481321139409409,
                1.7565907447102092,
                0.57599598816792164,
                -0.3539266261104293,
                5.066696053092949,
                -6.8948099505931779,
                0.50534444116741251,
                4.7400488079328378,
                3.943324176714444,
                6.2332242987377642,
                1.7003604679980404,
                -4.0980264896075074,
                1.4667077766627297,
                4.3700608579220335,
                3.5891351090271697,
                -1.8878010943161645,
                2.8729647120127413,
                -2.2280915130643884,
                -0.24784272086873907,
                -1.1763456931934448,
                0.81636793504759042,
                -2.3530192098391787,
                -0.20578007456941028,
                -1.3495477146604256,
                -5.7164767586646219,
                3.9792871409086432,
                4.2434848165130656,
                -2.8017036471973675,
                2.5819893825943931,
                -4.3577976051817124,
            ]
        )

        c_a = np.array(
            [
                -1.5169291265489147,
                -0.2082349479874758,
                0.63316058450527379,
                0.2996056386782382,
                2.2599078320743269,
                0.27704688274368583,
                -0.69222259288372689,
                0.0677073906142498,
                0.011450728354412288,
                0.0089055312102663126,
                -1.3925769602636826,
                -0.18558366388156874,
                -0.82758370434072914,
                -0.089692106043407951,
                0.3192792910197827,
                -0.11771969242248413,
                -1.3420886980883635,
                0.31838018203516122,
                0.64743426075914312,
                -0.956773006938416,
                -2.1859788152061275,
                0.14833878121282648,
                0.32104005128513557,
                -0.0865835932184397,
                2.3105491559235687,
                2.552543170508129,
                0.16665565805937393,
                -0.010145670768076966,
                0.10328504412153576,
                -0.042011374125897276,
                0.24139603871262802,
                -0.020468625284177838,
                1.0880402063222989,
                -0.15645607441918613,
                0.18452402484240943,
                -0.93886430218258821,
                -0.00057161499415276024,
                -0.00049109778979844856,
                0.63869928739219362,
                0.28794225569607923,
            ]
        )

        a = np.array(
            [
                -6.0818720398831614,
                -6.7711169389668449,
                2.04235120902932,
                -4.088439461456014,
                -4.0260407411298464,
                6.1912563758037011,
                2.8041200014844128,
                -1.5432379439829127,
                -1.0121103104932758,
                -0.045678138984986211,
                1.1674775717225221,
                1.0927203721987468,
                0.2065181283868481,
                -0.36479786388313684,
                3.5766998199950892,
                -5.2990973406978323,
                -3.9374320125379816,
                4.883815667566541,
                3.0918170278090544,
                5.3419825747853142,
            ]
        )

        b = np.zeros(20)
        xp1 = np.array([-0.70856474721809781, 0.31863289284106455])
        wrist_position = np.zeros(2)

        d = (joint_position_1 - -1.2795) * 0.701754385964912 - 1.0
        d1 = (joint_position_2 - -1.2795) * 0.701705143498702 - 1.0

        for k in range(20):
            b[k] = 2.0 / (np.exp(-2.0 * (a[k] + (b_a[k] * d + b_a[k + 20] * d1))) + 1.0) - 1.0

        for k in range(2):
            d = 0.0
            for i in range(20):
                d += c_a[k + (i << 1)] * b[i]
            wrist_position[k] = ((xp1[k] + d) - -1.0) / (-0.54567132576597 * float(k) + 1.63702291013563) + (
                -0.26178999999999997 * float(k) + -0.61087
            )

        # (wrist_position_roll, wrist_position_pitch)
        return wrist_position[0], wrist_position[1]

    def forward(
        self,
        joint_position_up_deg,
        joint_position_lower_deg,
        joint_velocity_up_deg=0,
        joint_velocity_lower_deg=0,
        joint_torque_up=0,
        joint_torque_lower=0,
    ):
        # wrist position, retarget
        joint_position_l = np.deg2rad(-1 * joint_position_lower_deg)
        joint_position_r = np.deg2rad(joint_position_up_deg)
        joint_velocity_l = np.deg2rad(-1 * joint_velocity_lower_deg)
        joint_velocity_r = np.deg2rad(joint_velocity_up_deg)
        joint_torque_l = -1 * joint_torque_lower
        joint_torque_r = joint_torque_up
        wrist_position_pitch, wrist_position_roll = self.fk_nnfit(joint_position_l, joint_position_r)

        rotation_matrix = radian_to_rotation_matrix(wrist_position_pitch, wrist_position_roll)
        target_rc1 = np.dot(rotation_matrix, self.initial_rc1)
        target_rc2 = np.dot(rotation_matrix, self.initial_rc2)
        target_ra1 = self.initial_ra1
        target_ra2 = self.initial_ra2

        # 根据角度得到旋转矩阵
        single_rotation_matrix_l = single_radian_rotation_matrix(joint_position_l)
        single_rotation_matrix_r = single_radian_rotation_matrix(joint_position_r)

        # 踝关节目标位置参数
        target_rb1 = target_ra1 + np.dot(single_rotation_matrix_l, (self.initial_rb1 - self.initial_ra1))
        target_rb2 = target_ra2 + np.dot(single_rotation_matrix_r, (self.initial_rb2 - self.initial_ra2))

        # bar和rod的向量表示
        r_bar1 = target_rb1 - target_ra1
        r_bar2 = target_rb2 - target_ra2
        r_rod1 = target_rc1 - target_rb1
        r_rod2 = target_rc2 - target_rb2

        # 旋量中的方向向量
        s11 = np.array([0, 1, 0])
        s21 = np.array([0, 1, 0])

        # Jx
        Jx = np.array(
            [
                r_rod1.tolist() + (np.cross(target_rc1, r_rod1)).tolist(),
                r_rod2.tolist() + (np.cross(target_rc2, r_rod2)).tolist(),
            ]
        )

        # J_theta
        J_theta = np.array([[np.dot(s11, np.cross(r_bar1, r_rod1)), 0], [0, np.dot(s21, np.cross(r_bar2, r_rod2))]])

        # G_matrix
        G_matrix = np.array(
            [[0, 0, 0, np.cos(wrist_position_roll), 0, -np.sin(wrist_position_roll)], [0, 0, 0, 0, 1, 0]]
        ).T

        # wrist velocity
        joint_velocity = np.array([joint_velocity_l, joint_velocity_r])
        wrist_velocity_pitch, wrist_velocity_roll = np.linalg.inv(
            np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix))
        ).dot(joint_velocity)

        # wrist torque
        joint_torque = np.array([joint_torque_l, joint_torque_r])
        wrist_torque_pitch, wrist_torque_roll = np.linalg.inv(
            np.linalg.inv(np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix))).T
        ).dot(joint_torque)

        if self.isLeft:
            return (
                np.rad2deg(wrist_position_roll),
                np.rad2deg(-1 * wrist_position_pitch),
                np.rad2deg(wrist_velocity_roll),
                np.rad2deg(-1 * wrist_velocity_pitch),
                (wrist_torque_roll),
                (-1 * wrist_torque_pitch),
            )  # numpy.floa64
        else:
            return (
                np.rad2deg(-1 * wrist_position_roll),
                np.rad2deg(wrist_position_pitch),
                np.rad2deg(-1 * wrist_velocity_roll),
                np.rad2deg(wrist_velocity_pitch),
                (-1 * wrist_torque_roll),
                (wrist_torque_pitch),
            )  # numpy.floa64

    def inverse(
        self,
        wrist_position_roll_deg,
        wrist_position_pitch_deg,
        wrist_velocity_roll_deg=0,
        wrist_velocity_pitch_deg=0,
        wrist_torque_roll=0,
        wrist_torque_pitch=0,
    ):
        if self.isLeft:
            wrist_position_pitch = np.deg2rad(-1 * wrist_position_pitch_deg)
            wrist_position_roll = np.deg2rad(wrist_position_roll_deg)
            wrist_velocity_pitch = np.deg2rad(-1 * wrist_velocity_pitch_deg)
            wrist_velocity_roll = np.deg2rad(wrist_velocity_roll_deg)
            wrist_torque_pitch = -1 * wrist_torque_pitch
            wrist_torque_roll = wrist_torque_roll
        else:
            wrist_position_pitch = np.deg2rad(wrist_position_pitch_deg)
            wrist_position_roll = np.deg2rad(-1 * wrist_position_roll_deg)
            wrist_velocity_pitch = np.deg2rad(wrist_velocity_pitch_deg)
            wrist_velocity_roll = np.deg2rad(-1 * wrist_velocity_roll_deg)
            wrist_torque_pitch = wrist_torque_pitch
            wrist_torque_roll = -1 * wrist_torque_roll

        rotation_matrix = radian_to_rotation_matrix(wrist_position_pitch, wrist_position_roll)

        # 旋转后的关节点
        target_rc1 = np.dot(rotation_matrix, self.initial_rc1)
        target_rc2 = np.dot(rotation_matrix, self.initial_rc2)
        target_ra1 = self.initial_ra1
        target_ra2 = self.initial_ra2

        # 得到计算公式的元素
        interm_a1 = target_rc1 - target_ra1
        a1 = interm_a1[0]
        interm_a2 = target_rc2 - target_ra2
        a2 = interm_a2[0]
        interm_b1 = target_ra1 - target_rc1
        b1 = interm_b1[2]
        interm_b2 = target_ra2 - target_rc2
        b2 = interm_b2[2]

        # 计算二阶范数
        norm_1 = np.linalg.norm(target_rc1 - target_ra1, ord=2)
        norm_2 = np.linalg.norm(target_rc2 - target_ra2, ord=2)

        c1 = (self.length_rod1**2 - self.length_bar1**2 - norm_1**2) / (2 * self.length_bar1)
        c2 = (self.length_rod2**2 - self.length_bar2**2 - norm_2**2) / (2 * self.length_bar2)

        # motor position
        joint_position_l = asin(
            (b1 * c1 + sqrt(b1**2 * c1**2 - (a1**2 + b1**2) * (c1**2 - a1**2))) / (a1**2 + b1**2)
        ) - np.deg2rad(self.initial_offset)
        joint_position_r = asin(
            (b2 * c2 + sqrt(b2**2 * c2**2 - (a2**2 + b2**2) * (c2**2 - a2**2))) / (a2**2 + b2**2)
        ) - np.deg2rad(self.initial_offset)

        single_rotation_matrix_l = single_radian_rotation_matrix(joint_position_l)
        single_rotation_matrix_r = single_radian_rotation_matrix(joint_position_r)

        # 踝关节目标位置参数
        target_rb1 = target_ra1 + np.dot(single_rotation_matrix_l, (self.initial_rb1 - self.initial_ra1))
        target_rb2 = target_ra2 + np.dot(single_rotation_matrix_r, (self.initial_rb2 - self.initial_ra2))

        # bar和rod的向量表示
        r_bar1 = target_rb1 - target_ra1
        r_bar2 = target_rb2 - target_ra2
        r_rod1 = target_rc1 - target_rb1
        r_rod2 = target_rc2 - target_rb2

        # 旋量中的方向向量
        s11 = np.array([0, 1, 0])
        s21 = np.array([0, 1, 0])

        # 雅可比矩阵的组成部分
        # Jx
        Jx = np.array(
            [
                r_rod1.tolist() + (np.cross(target_rc1, r_rod1)).tolist(),
                r_rod2.tolist() + (np.cross(target_rc2, r_rod2)).tolist(),
            ]
        )

        # J_theta
        J_theta = np.array([[np.dot(s11, np.cross(r_bar1, r_rod1)), 0], [0, np.dot(s21, np.cross(r_bar2, r_rod2))]])

        # G_matrix
        G_matrix = np.array(
            [[0, 0, 0, np.cos(wrist_position_pitch), 0, -np.sin(wrist_position_pitch)], [0, 0, 0, 0, 1, 0]]
        ).T

        # motor velocity
        wrist_velocity = np.array([wrist_velocity_pitch, wrist_velocity_roll])
        joint_velocity_l, joint_velocity_r = np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix)).dot(wrist_velocity)

        # motor torque
        wrist_torque = np.array([wrist_torque_pitch, wrist_torque_roll])
        joint_torque_l, joint_torque_r = np.linalg.inv(np.dot(np.linalg.inv(J_theta), np.dot(Jx, G_matrix))).T.dot(
            wrist_torque
        )

        return (
            np.rad2deg(joint_position_r),  # the upper motor
            np.rad2deg(-1 * joint_position_l),  # the lower motor
            np.rad2deg(joint_velocity_r),
            np.rad2deg(-1 * joint_velocity_l),
            (joint_torque_r),
            (-1 * joint_torque_l),
        )


class PoseSolver:
    def __init__(self, joints_name, encoders_state, encoders_configuration, joints_configuration, joints_current_pose, dtype=np.float32):
        self.parallel_ankle_left    = ParallelAnkle("left")
        self.parallel_ankle_right   = ParallelAnkle("right")
        self.parallel_wrist_left    = ParallelWrist("left")
        self.parallel_wrist_right   = ParallelWrist("right")
        self.parallel_head          = ParallelHead()
        self.dtype                  = dtype
        self.encoders_configuration = encoders_configuration
        self.joints_configuration   = joints_configuration
        self.encoders_state         = encoders_state
        self.num_joints             = len(joints_configuration)
        self.encoders_name          = list(encoders_state.keys())
        self.joints_name            = joints_name
        self.encoders_related_joints_name = [self.encoders_state[name]["joint_name"] for name in self.encoders_name]
        self.encoder_to_joint_index = [self.joints_name.index(joint_name) for joint_name in self.encoders_related_joints_name]
        self.encoders_poweron_pose     = np.array([self.encoders_state[name]["poweron_pose"] for name in self.encoders_name], dtype=dtype)
        self.encoders_calibration_pose = np.array([self.encoders_state[name]["calibration_pose"] for name in self.encoders_name], dtype=dtype)
        self.encoder_to_joint          = {name: self.encoders_state[name]["joint_name"] for name in self.encoders_name}
        self.encoders_direction        = np.array([encoders_configuration[name]["direction"] for name in self.encoders_name], dtype=dtype)
        self.encoders_reduction_ratio  = np.array([encoders_configuration[name]["reduction_ratio"] for name in self.encoders_name], dtype=dtype)
        self.encoders_initial_pose     = (((self.encoders_poweron_pose - self.encoders_calibration_pose + 180) % 360 - 180) * self.encoders_direction / self.encoders_reduction_ratio).astype(dtype)
        self.joints_kinematic_reduction_ratio = np.array([joints_configuration[joint_name]["kinematic_reduction_ratio"] for joint_name in self.joints_name], dtype=dtype)
        self.joints_kinetic_reduction_ratio   = np.array([joints_configuration[joint_name]["kinetic_reduction_ratio"] for joint_name in self.joints_name], dtype=dtype)
        self.joints_direction                 = np.array([joints_configuration[joint_name]["direction"] for joint_name in self.joints_name], dtype=dtype)

        joints_current_pose_list       = np.array([joints_current_pose[name].position for name in self.joints_name], dtype=self.dtype)
        self.joints_initial_pose       = np.zeros(self.num_joints, dtype=dtype)
        self.joints_initial_pose[self.encoder_to_joint_index] = -1 * (self.encoders_initial_pose - joints_current_pose_list[self.encoder_to_joint_index] / self.joints_kinematic_reduction_ratio[self.encoder_to_joint_index] * self.joints_direction[self.encoder_to_joint_index])
        # print(f"self.joints_initial_pose = {self.joints_initial_pose}")

    def solve(self, joints_realtime_pose_angle, joints_realtime_velocity, joints_realtime_current):
        joints_realtime_pose_angle_list = np.array([joints_realtime_pose_angle[name] for name in self.joints_name], dtype=self.dtype)
        joints_velocity_list            = np.array([joints_realtime_velocity[name] for name in self.joints_name], dtype=self.dtype)
        joints_realtime_current_list    = np.array([joints_realtime_current[name] for name in self.joints_name], dtype=self.dtype)
        joints_position  = joints_realtime_pose_angle_list / self.joints_kinematic_reduction_ratio * self.joints_direction - self.joints_initial_pose

        joints_velocity = joints_velocity_list / self.joints_kinematic_reduction_ratio * self.joints_direction
        joints_kinetic  = joints_realtime_current_list * self.joints_kinetic_reduction_ratio * self.joints_direction

        (
            joints_position[4],
            joints_position[5],
            joints_velocity[4],
            joints_velocity[5],
            joints_kinetic[4],
            joints_kinetic[5],
        ) = self.parallel_ankle_left.forward(
            joint_position_up_deg=joints_position[4],
            joint_position_lower_deg=joints_position[5],
            joint_velocity_up_deg=joints_velocity[4],
            joint_velocity_lower_deg=joints_velocity[5],
            joint_torque_up=joints_kinetic[4],
            joint_torque_lower=joints_kinetic[5],
        )

        (
            joints_position[10],
            joints_position[11],
            joints_velocity[10],
            joints_velocity[11],
            joints_kinetic[10],
            joints_kinetic[11],
        ) = self.parallel_ankle_right.forward(
            joint_position_up_deg=joints_position[10],
            joint_position_lower_deg=joints_position[11],
            joint_velocity_up_deg=joints_velocity[10],
            joint_velocity_lower_deg=joints_velocity[11],
            joint_torque_up=joints_kinetic[10],
            joint_torque_lower=joints_kinetic[11],
        )

        (
            joints_position[23],
            joints_position[24],
            joints_velocity[23],
            joints_velocity[24],
            joints_kinetic[23],
            joints_kinetic[24],
        ) = self.parallel_wrist_left.forward(
            joint_position_up_deg=joints_position[23],
            joint_position_lower_deg=joints_position[24],
            joint_velocity_up_deg=joints_velocity[23],
            joint_velocity_lower_deg=joints_velocity[24],
            joint_torque_up=joints_kinetic[23],
            joint_torque_lower=joints_kinetic[24],
        )

        (
            joints_position[30],
            joints_position[31],
            joints_velocity[30],
            joints_velocity[31],
            joints_kinetic[30],
            joints_kinetic[31],
        ) = self.parallel_wrist_right.forward(
            joint_position_up_deg=joints_position[30],
            joint_position_lower_deg=joints_position[31],
            joint_velocity_up_deg=joints_velocity[30],
            joint_velocity_lower_deg=joints_velocity[31],
            joint_torque_up=joints_kinetic[30],
            joint_torque_lower=joints_kinetic[31],
        )

        (
            joints_position[15],
            joints_position[16],
            joints_velocity[15],
            joints_velocity[16],
            joints_kinetic[15],
            joints_kinetic[16],
        ) = self.parallel_head.forward(
            joint_position_l_deg=joints_position[15],
            joint_position_r_deg=joints_position[16],
            joint_velocity_l_deg=joints_velocity[15],
            joint_velocity_r_deg=joints_velocity[16],
            joint_torque_l=joints_kinetic[15],
            joint_torque_r=joints_kinetic[16],
        )
        return joints_position.astype(self.dtype), joints_velocity.astype(self.dtype), joints_kinetic.astype(self.dtype)

    def _calculate_parallel_ankle_inverse_kinematic(
        self,
        left_leg_ankle_position_pitch_deg,
        left_leg_ankle_position_roll_deg,
        right_leg_ankle_position_pitch_deg,
        right_leg_ankle_position_roll_deg,
    ):
        """
        Calculate parallel ankle inverse kinematic position

        Input:
        - left_leg_ankle_position_pitch_deg
        - left_leg_ankle_position_roll_deg
        - right_leg_ankle_position_pitch_deg
        - right_leg_ankle_position_roll_deg
        """

        # parallel ankle
        (
            left_leg_ankle_position_joint_upper_deg,
            left_leg_ankle_position_joint_lower_deg,
            _,
            _,
            _,
            _,
        ) = self.parallel_ankle_left.inverse(
            ankle_position_pitch_deg=left_leg_ankle_position_pitch_deg,
            ankle_position_roll_deg=left_leg_ankle_position_roll_deg,
        )
        (
            right_leg_ankle_position_joint_upper_deg,
            right_leg_ankle_position_joint_lower_deg,
            _,
            _,
            _,
            _,
        ) = self.parallel_ankle_right.inverse(
            ankle_position_pitch_deg=right_leg_ankle_position_pitch_deg,
            ankle_position_roll_deg=right_leg_ankle_position_roll_deg,
        )
        return (
            left_leg_ankle_position_joint_upper_deg,
            left_leg_ankle_position_joint_lower_deg,
            right_leg_ankle_position_joint_upper_deg,
            right_leg_ankle_position_joint_lower_deg,
        )

    def _calculate_parallel_wrist_inverse_kinematic(
        self,
        left_leg_wrist_position_pitch_deg,
        left_leg_wrist_position_roll_deg,
        right_leg_wrist_position_pitch_deg,
        right_leg_wrist_position_roll_deg,
    ):
        """
        Calculate parallel wrist inverse kinematic position

        Input:
        - left_leg_wrist_position_pitch_deg
        - left_leg_wrist_position_roll_deg
        - right_leg_wrist_position_pitch_deg
        - right_leg_wrist_position_roll_deg
        """

        # parallel wrist
        (
            left_leg_wrist_position_joint_upper_deg,
            left_leg_wrist_position_joint_lower_deg,
            _,
            _,
            _,
            _,
        ) = self.parallel_wrist_left.inverse(
            wrist_position_pitch_deg=left_leg_wrist_position_pitch_deg,
            wrist_position_roll_deg=left_leg_wrist_position_roll_deg,
        )
        (
            right_leg_wrist_position_joint_upper_deg,
            right_leg_wrist_position_joint_lower_deg,
            _,
            _,
            _,
            _,
        ) = self.parallel_wrist_right.inverse(
            wrist_position_pitch_deg=right_leg_wrist_position_pitch_deg,
            wrist_position_roll_deg=right_leg_wrist_position_roll_deg,
        )
        return (
            left_leg_wrist_position_joint_upper_deg,
            left_leg_wrist_position_joint_lower_deg,
            right_leg_wrist_position_joint_upper_deg,
            right_leg_wrist_position_joint_lower_deg,
        )

    def inverse(self, target_position):
        joint_control_position = target_position.copy()
        (
            joint_control_position[4],
            joint_control_position[5],
            joint_control_position[10],
            joint_control_position[11],
        ) = self._calculate_parallel_ankle_inverse_kinematic(
            joint_control_position[4],
            joint_control_position[5],
            joint_control_position[10],
            joint_control_position[11],
        )

        (
            joint_control_position[24],
            joint_control_position[23],
            joint_control_position[31],
            joint_control_position[30],
        ) = self._calculate_parallel_wrist_inverse_kinematic(
            joint_control_position[24],
            -joint_control_position[23],
            joint_control_position[31],
            -joint_control_position[30],
        )

        (joint_control_position[15], joint_control_position[16], _, _, _, _) = self.parallel_head.inverse(
            joint_control_position[15], joint_control_position[16]
        )
        return (joint_control_position + self.joints_initial_pose) * self.joints_kinematic_reduction_ratio * self.joints_direction


