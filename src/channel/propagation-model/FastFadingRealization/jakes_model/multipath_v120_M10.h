/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010,2011,2012,2013 TELEMATICS LAB, Politecnico di Bari
 *
 * This file is part of LTE-Sim
 *
 * LTE-Sim is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation;
 *
 * LTE-Sim is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LTE-Sim; if not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Giuseppe Piro <g.piro@poliba.it>
 */



#ifndef MULTIPATH_V120_M10_H_
#define MULTIPATH_V120_M10_H_

static float multipath_M10_v_120[3000] = {
 8.54016, 9.93546, 11.1318, 2.0929, 8.13349, 2.89519, 7.206, 5.46443, 4.93812, 5.62575, 2.86327, 6.00864, -2.98311, 5.74984, -2.10588, 5.22904, 1.90621, 4.41125, 3.87692, 2.34765, 4.21269, 0.684186, 5.33258, -1.66778, 3.03785, -6.09869, 6.13708, 5.49625, -3.66913, 2.79913, -14.5569, -11.6559, 4.99606, 6.12354, 0.734493, 8.85585, 8.93465, 2.47981, 5.63929, 6.57943, 4.59478, -1.30061, 3.85538, 7.2592, 5.47064, 5.16247, 8.77602, 4.51872, 7.66059, 8.56782, -0.571848, 8.93865, 6.81705, 5.34266, 7.48482, -2.86467, 7.97611, 6.78178, -1.10495, 4.96402, -0.641248, 2.15095, 0.34023, 1.31887, 4.25742, 0.710901, 4.14818, 7.7919, 7.50907, -0.401172, 5.933, 6.27309, 0.381589, -3.50286, -0.904366, -4.38209, 5.73362, 7.57867, -8.52928, 8.96629, 8.82396, 2.70252, 9.45529, 6.36917, 7.27624, 8.9422, 2.46031, 4.8804, 0.432302, 3.31186, -0.692771, 2.04347, -6.46393, 5.06737, -2.6241, 7.32532, 6.69993, 6.53655, 10.2998, 7.03696, 7.13486, 8.07355, 1.72699, 7.38174, 0.0900457, 9.1711, 6.83531, 7.46749, 9.81347, 4.42267, 7.48697, 8.16295, 3.02932, 0.455617, 0.75203, 0.466316, 0.930287, -1.60116, 5.83653, 6.21224, -0.554653, 3.14528, 2.76219, -1.52521, -1.00658, -4.33885, 3.72802, 6.19712, 0.553921, 5.49429, 3.77843, 6.8311, 9.27148, 2.49331, 9.28307, 9.99056, 0.644923, 8.3845, 7.3912, 0.42262, 5.19712, -6.24782, 4.059, -3.5116, 6.55401, 6.25239, -4.38849, 3.5811, 2.71555, -4.32491, -0.66015, 3.94232, 5.99672, 4.30303, 3.12165, 7.8398, 6.44958, 3.45835, 8.62093, 7.51067, -2.90537, 6.25065, 2.60688, 4.7557, 5.85814, -2.84549, 6.8984, 6.26006, -16.2491, 3.57671, -1.26065, -4.01936, 2.93606, 6.089, -5.0854, 8.52016, 9.56999, 2.03707, 8.53638, 8.21884, 3.0645, 8.71511, 5.08081, 5.82822, 5.88044, 3.6006, 7.16529, -2.55772, 7.1368, 6.21158, 3.79769, 8.47775, 7.49401, -1.74554, 5.4104, 6.89698, 5.33131, -6.33553, 5.19052, 3.76241, 3.10638, 6.63318, 1.62927, 6.08066, 7.13243, -3.58272, 6.75506, 7.43179, 4.81469, -1.43753, -1.06909, 4.10256, 5.31638, -0.324429, 5.31491, 6.81785, 2.05878, 3.32952, 3.70004, -7.22385, -1.69901, -5.94775, 1.41911, 5.7913, 5.41914, -1.1822, 6.3861, 4.97581, -5.99299, -4.49098, 4.09989, 4.19453, 3.69501, 7.94894, 4.40731, 6.96953, 8.76691, 1.82411, 7.32099, 6.13098, 6.27773, 9.70709, 6.36859, 7.64724, 9.99052, 5.63722, 7.85099, 9.56997, 5.09564, 5.80487, 6.67356, -4.03648, 3.85706, 1.03133, -13.6517, 2.80877, 5.64012, -1.628, 7.00166, 8.14303, 2.58044, 4.75954, 3.66394, 0.116578, -0.604135, 3.88014, 4.25835, 3.15041, 7.53825, 5.13455, 1.17073, 1.64386, 2.89126, 3.85718, 1.24825, 5.20239, -8.33683, 4.7947, -0.718252, 4.62251, -0.667595, 7.44527, 8.31375, -1.21631, 9.14666, 7.73574, 5.28245, 9.33019, 6.5873, 4.8412, 8.45168, 7.84145, 4.12803, 3.04084, 7.53141, 6.71429, 0.41504, 7.3144, 4.41646, 4.07346, 2.49898, 6.92977, 8.65854, -5.734, 8.70213, 6.97184, 6.00919, 8.06054, -0.457529, 8.04872, 1.30985, 8.36436, 7.88484, 3.23337, 7.43796, -9.49686, 7.0471, 0.582618, 7.45514, 6.43382, 5.21674, 7.9559, -7.25013, 8.76276, 8.31256, -14.6226, 6.67607, 6.53002, 4.13006, -4.01406, 5.40008, 8.70879, 8.08875, -4.9131, 7.8609, 6.65446, 1.87287, 6.52266, 1.38259, 4.58778, 4.60304, -0.441013, 6.21556, 6.05328, -0.679361, 4.27608, 5.30718, -5.17716, 5.06532, 3.44404, 4.77567, 7.54181, 1.21036, 6.82401, 5.88096, 5.7313, 8.91757, 3.61856, 7.54657, 7.75288, -2.29421, 6.45089, 3.03278, -11.4323, 5.27082, 7.92943, 1.67988, 8.49154, 9.46467, 1.24207, 8.13536, 8.22644, 1.50616, 3.35035, 3.0572, -1.78297, -0.811871, 4.30482, 4.20994, 2.10219, 7.99435, 6.61348, 4.4358, 7.97256, -2.01129, 8.1911, 7.25489, 3.61385, 7.11675, -10.4926, 6.56241, 1.6211, 5.38415, 2.56843, 6.00649, 7.12545, -0.206035, 0.235428, 0.990901, -1.39008, 7.45207, 9.24702, -17.8652, 10.5668, 10.2959, 2.55024, 9.87849, 5.76824, 7.27371, 5.80511, 6.61968, 7.49573, 3.76377, 7.73145, 2.39933, 9.63112, 6.12277, 8.5737, 9.26533, 3.97738, 10.1847, 6.82635, 7.74345, 8.79025, -2.32657, 8.49725, 7.28776, -5.25003, 2.41502, 0.844462, 0.843273, 1.5951, -3.6139, -3.5806, -6.71322, -11.5557, 5.55908, 8.16953, 4.7205, 7.40392, 9.78714, 6.26248, 5.87683, 7.49233, -0.525426, 5.00605, 4.68495, 1.58754, 0.57823, -7.46556, 3.75994, 4.68879, -0.454764, 6.95019, 5.57173, 1.70414, 5.17123, -4.96691, 5.75308, 3.91909, -1.20096, 0.912305, -15.4322, 0.632694, 5.57163, 3.31098, 5.78509, 8.79, 6.20884, 4.24873, 7.452, 4.83165, -13.6443, -6.22636, 2.8447, 2.60859, -3.49434, 4.38488, 4.55393, -6.8379, 6.89846, 8.17054, 1.83494, 8.1377, 9.03293, -1.3664, 10.1498, 9.43892, 3.89461, 9.62864, 5.27708, 7.8021, 7.83514, 3.16661, 7.87332, 1.18751, 5.72132, 2.25686, 3.37508, -4.088, 7.81266, 6.48061, 6.90778, 10.1871, 7.09201, 5.29924, 6.42958, -2.20846, 3.25042, 2.31021, 4.76783, 2.91796, 7.85051, 4.44021, 5.27686, 5.48088, 2.30174, 5.7421, -11.9209, 4.06028, 1.22907, 7.57224, 4.24879, 7.17024, 8.8484, 0.991959, 7.84909, 7.82183, -10.086, 6.23639, 4.89784, -0.617728, 5.6103, 4.94651, 0.106223, 8.26064, 9.41823, 5.34747, 7.33595, 9.38621, 4.5882, 6.8115, 7.42897, -4.22924, 3.32851, 0.806206, 6.18069, -0.0151829, 6.3225, 5.78506, 3.9902, 8.03745, 4.52716, 5.01212, 6.24927, -4.46673, 3.20315, -10.0135, 3.61342, -2.58272, 5.58595, 6.58153, -5.09108, 6.72716, 7.15831, 0.90981, 3.32665, 0.256895, 4.52572, 6.55663, 2.05398, 4.92135, 7.0286, 4.53084, 0.804718, 6.46473, 7.57862, 6.37141, -6.27067, 7.49283, 8.08609, -5.31872, 8.55052, 8.59739, -1.58859, 8.54623, 6.40231, 5.07596, 6.93873, 0.497417, 7.38992, 0.250381, 7.9888, 7.73193, 3.84123, 9.201, 7.1252, 1.61839, 4.26347, 1.35046, 5.24471, -6.40098, 4.9932, 3.46906, -0.170459, 2.66377, -4.44414, -12.0691, 3.25593, 5.72304, 3.90173, -0.474332, 5.82128, 6.43012, 2.26463, 5.36039, 8.43719, 6.40636, 5.37838, 9.44105, 7.26165, 5.5044, 9.05266, 5.82138, 4.9963, 7.0861, 3.071, -6.78955, 0.532102, 4.58085, 2.79091, -3.6474, 1.59375, 1.11037, 1.56523, 0.638661, 0.213221, 5.64279, 5.11946, -8.08087, 3.145, -4.90765, 5.51012, 3.62135, 4.7652, 7.79113, 4.08724, 5.89682, 8.0167, 3.9397, 4.92548, 6.80939, 3.17648, -0.159533, 1.14082, -5.22151, -1.29666, 2.7081, -10.1527, 7.52707, 9.50282, 6.01555, 7.67821, 9.9574, 4.6576, 8.36646, 8.68453, 1.51539, 8.55736, 2.49128, 8.74361, 9.07871, -2.84809, 8.35319, 5.76911, 4.02346, 4.76863, 1.41722, 4.61079, -3.39659, 4.64568, 0.414557, 0.702232, -3.49946, -5.0184, 3.60717, 6.69762, 1.03302, 7.07418, 7.26243, 4.23174, 9.76976, 8.31772, 2.05661, 7.64346, 5.42064, -1.79183, -1.6128, -11.7935, 5.87364, 7.80975, 3.58381, 5.21946, 5.57555, -0.229764, 4.96934, -9.18863, 4.73785, 2.8367, -3.68198, -5.64302, -5.77025, 3.19487, 6.05362, -9.86933, 8.20393, 8.1609, 2.4579, 8.59023, 1.81251, 8.86646, 8.42563, 4.77457, 8.76698, -2.68706, 9.89335, 7.92186, 8.12491, 10.8037, 5.7315, 8.8379, 9.88095, 5.438, 2.24372, 3.11942, 1.81625, 3.94584, 3.50581, -1.44387, 4.68882, 1.1592, 2.7208, 3.60062, -8.39878, 2.91392, 1.29452, -3.9743, 2.9446, 5.00638, 5.01819, -0.247867, 3.04968, 3.58061, -1.26235, 5.2474, 4.03963, -11.7069, -0.40805, -1.02592, 2.23139, 3.85277, -8.42077, 5.86407, 6.56588, 0.0154569, 0.849921, 0.19524, 3.94701, 4.03903, 9.53648, 8.03219, 7.13778, 10.8063, 6.91898, 8.84505, 10.0931, -0.404298, 8.48488, 6.25453, 5.70487, 7.98024, 4.08039, -1.51301, -1.72297, -3.36558, 1.20647, -4.08581, 4.18846, 5.08259, -3.27565, 5.49864, 2.29633, 2.38451, 1.54628, -2.00251, 1.27112, 7.59628, 6.42507, 6.00522, 9.66059, 6.10015, 6.32113, 6.27386, 4.04047, 5.80423, 5.50112, 8.89328, -5.68245, 9.96777, 9.46698, 2.63234, 8.68737, 2.32869, 7.15344, 5.83776, 0.571404, -0.550246, 3.83315, -3.67777, 7.20987, 6.21575, 7.02448, 9.81735, 0.337758, 9.96183, 8.81053, 7.72002, 11.0652, 6.24785, 9.00932, 9.51364, -1.26126, 5.48716, -5.38035, 5.52967, -10.9511, 7.29016, 6.4113, 2.30565, 7.47579, 6.48281, 2.3278, -5.53627, -2.861, 1.04126, -1.29168, -0.161997, 1.73587, -6.48037, -5.40068, 0.00131378, 0.324753, 2.65996, 5.65743, -10.3224, 7.81512, 7.68971, 1.87954, 8.48755, 5.26056, 6.65798, 7.9839, -3.34607, 8.56509, 8.26894, 0.361677, 5.51958, 6.90264, 5.95508, -0.678071, 6.07464, 8.59135, 6.92273, -1.17425, 5.27934, -1.43818, 3.35329, -1.58666, 3.98865, 3.03916, 2.85849, 5.93148, 1.56377, 3.13758, 4.25572, -1.24118, 1.24932, 4.38803, 4.93702, 0.563424, 4.30445, 6.41868, 1.30081, 4.05269, 1.75046, 5.07695, 6.32528, -3.03472, 6.20083, 0.841833, 5.07603, 1.05847, 5.98009, 2.81388, 9.0794, 10.9911, 3.85386, 10.7324, 10.9957, -0.16151, 10.5508, 8.27806, 6.27748, 8.07129, -1.12241, 7.16101, 0.960932, 5.41358, 2.14701, 4.68606, 4.56758, 0.0188511, 3.61066, -5.62568, -5.96024, 5.36362, 6.0933, -0.857844, 7.20365, 4.35529, 5.5063, 6.92401, -10.2409, 5.86655, -0.52441, 6.76669, 7.76185, 3.48617, 1.99348, 4.59425, 5.03571, 5.05406, 0.291767, 5.09025, 7.64344, 5.63037, 0.214237, 5.76244, 4.88725, -0.728187, 1.72886, 5.42236, 5.38779, -4.3912, 7.45977, 8.01184, -0.742379, 6.73204, 5.29753, 4.69493, 7.38995, -4.62181, 7.58417, 6.23102, 5.78779, 9.37414, 6.70858, 4.70621, 7.11464, -7.50314, 6.74732, 4.56082, 5.7592, 8.49432, 4.32995, 6.88137, 8.93257, 5.79941, 3.16032, 6.129, 3.7667, -1.00626, -2.0783, -3.14307, 4.83215, 5.08102, 1.90307, 8.45427, 7.9866, -2.46764, 7.61277, 5.7923, 2.01543, 5.18459, -0.558072, -7.03641, 3.19927, 5.28099, -1.43267, 2.98149, -4.85124, 4.37129, -0.3035, 6.75801, 7.79112, -2.73136, 8.59623, 7.38096, 3.02866, 6.80428, -4.06205, 7.15123, 2.51147, 7.74273, 8.93162, 0.304596, 8.07908, 8.09375, -4.17963, 8.35403, 8.71103, 3.9456, 5.64935, 8.0083, 5.3891, 3.82894, 7.75241, 6.35601, -4.22521, 0.286564, -4.47423, -1.2394, 1.89827, 4.94714, -0.139456, 3.47089, 2.44044, 2.08013, 3.4574, 3.05124, 7.60705, 5.27585, 4.88866, 8.01136, 5.28048, 0.978565, 3.35477, -5.51486, 2.65525, -1.79381, -0.198072, -3.80161, 2.55579, 5.025, 3.31726, -2.12626, 4.70925, 4.5865, -11.8937, 6.11174, 7.91236, 6.36328, -0.804286, 7.78674, 8.55067, 4.23285, 6.33716, 8.57372, 4.3253, 6.41717, 7.33024, 1.55325, 8.75226, 6.21508, 7.16654, 9.53214, 4.3837, 6.50827, 4.89081, 5.35925, 6.22312, 3.66792, 7.5468, -2.9683, 9.09317, 7.7707, 5.36274, 8.65932, 0.119295, 7.7146, 6.3128, 2.36415, 3.52647, 4.45058, 6.34364, 1.46615, 8.49101, 7.32437, 0.0192852, 6.92816, 6.61218, 4.81978, 0.136639, 4.57427, 7.90783, 5.77194, 4.84741, 7.43007, -1.31325, 9.03594, 7.91214, 4.52915, 8.79292, 4.54726, 6.12346, 7.06628, 2.2589, -20.0451, -2.96616, -1.85262, 4.46391, 0.809097, 5.66523, 7.03503, -6.65943, 6.17548, 2.96477, 3.55082, -2.52071, 7.08397, 6.74327, 5.89374, 9.98967, 6.87747, 7.05414, 8.72475, 1.56963, 3.65409, 1.68763, 7.19898, 1.35305, 7.25, 6.63127, 5.18488, 8.81269, 3.59121, 7.46609, 7.60756, 2.46815, 8.9541, 7.78228, 0.666801, 8.03189, 7.16048, -0.911379, 7.90764, 7.60849, -4.1009, 7.95655, 7.63212, -7.63912, 5.80055, 4.16127, -10.1448, -1.29286, 4.06555, -1.32011, 5.90789, 7.53945, 4.48546, -10.6377, -7.96551, -4.03806, 3.6773, 6.16859, -10.569, 7.8915, 7.65605, 1.96921, 7.8318, 1.17571, 7.59007, 6.97167, 2.78719, 6.57735, -11.9038, 5.08114, -7.0805, 6.05802, -0.768844, 7.04636, 5.65186, 6.57129, 9.06662, 0.0709912, 9.05609, 9.07648, -17.4131, 8.19217, 8.43306, 6.27667, 1.63129, 2.5134, 6.31155, 3.87991, 4.18994, 5.92418, 0.50644, 6.64843, -2.09879, 9.23363, 8.07211, 6.64306, 9.96524, 1.95355, 9.54902, 8.45549, 6.35498, 9.43852, 0.793695, 7.85078, 4.65237, 5.99655, 4.91282, 5.14941, 5.68536, 4.80618, 7.46087, 1.20836, 8.66337, 3.1174, 9.38247, 10.289, 2.89527, 7.55873, 6.55085, -4.99334, -2.59725, 3.38977, 1.43972, 8.59701, 7.92611, 3.30285, 8.59521, 4.0562, 6.90628, 6.80219, 3.00428, 7.49409, 2.09946, 5.86182, 5.71678, 0.0301989, 6.27676, 4.54367, 0.634921, 6.07564, 5.22428, 0.0132708, 7.3621, 7.02111, -2.05629, 6.85112, 1.22854, 8.129, 9.26235, -0.567331, 8.86331, 8.82385, -3.90936, 6.73934, 5.86023, 2.60736, 4.01958, 4.36791, 0.912652, 7.67963, 6.83042, 1.08011, 7.1504, 4.60619, 2.3437, 5.0904, 1.81838, -5.37746, 0.149679, 1.34039, -1.06998, 1.01021, 4.2431, -0.0538352, 2.29936, -1.21128, 5.67515, 7.541, 2.54406, 5.06497, 4.82994, 0.0765643, 3.8104, -1.84141, 4.77319, -0.0249011, 3.85984, 5.37053, 4.7749, 4.72307, -0.453997, 7.16616, 9.86349, 6.62232, 8.49671, 10.6678, 4.09403, 9.60615, 9.38559, 3.22894, 8.89226, 1.50741, 7.85991, 5.45499, 6.13523, 5.83713, 6.7685, 9.46729, 3.04149, 7.80709, 6.13241, 6.05926, 7.67555, 1.07749, 8.42406, 5.31807, 6.09235, 7.76023, 2.57323, 1.52981, -3.60684, 0.279631, -5.96286, 4.1218, 4.78299, 2.63942, -3.53758, 2.24195, 6.50149, 6.65131, -3.52957, 8.41821, 8.78593, -0.948757, 7.86582, 7.62314, -1.11912, 7.99535, 7.93173, 3.55822, 1.64972, 4.7077, 3.39384, -9.80384, 3.18232, 1.85953, 0.811468, 4.08643, -16.7767, 5.88286, 6.32128, -2.71197, 4.61337, 3.95264, -5.98756, -5.55229, 3.98254, 4.49018, -1.11389, 6.76292, 6.3542, -3.70379, 6.55395, 6.11654, -1.34479, -0.344882, -1.53212, 3.55869, -3.89535, 4.51993, 2.83435, 6.1504, 9.21984, 5.70552, 8.2219, 10.4605, 5.57778, 8.9129, 10.0263, 1.80643, 8.10679, 6.6056, 4.66646, 7.32985, -3.3764, 6.4997, 5.8362, -1.1456, -8.47396, -1.69895, 1.49913, 6.46409, 4.81913, 4.24152, 7.686, 4.60747, 1.07769, -2.85481, 4.87715, 4.28607, 3.36004, 6.34809, -5.71451, 7.72825, 7.03681, 1.32205, 7.41784, 5.53264, -3.24056, 2.72436, 1.40591, 3.22237, 5.36001, 3.15161, 3.54268, 7.3687, 6.19266, -6.83098, 5.26773, 4.2198, -0.205789, 5.8912, 4.50355, 3.18149, 7.78539, 5.48455, 6.54856, 10.0634, 8.24787, 4.14371, 8.77818, 5.21676, 5.9631, 7.33462, -2.40572, 5.16443, 1.72657, 3.78772, 3.78635, 2.0957, 6.29895, 2.82512, 4.87368, 7.48119, 5.3584, 0.638383, 6.38043, 6.36525, 3.57097, -5.0903, -0.738895, 2.58538, 3.92401, 1.28918, 2.38397, 5.82415, 3.09624, 2.31419, 3.15953, 3.81503, 7.98008, 6.0758, 3.13499, 6.94362, 3.69349, -0.162735, 0.876863, -0.270065, 4.53315, 5.36715, 1.7517, 8.78002, 7.33887, 6.79691, 10.2956, 5.90995, 8.60652, 9.17141, 0.208174, 7.96934, -2.98213, 9.50289, 7.47269, 7.88813, 10.3353, 4.01919, 8.82097, 9.01397, 0.611732, 4.00479, -5.87234, 3.14134, 0.49424, -10.3896, 0.150003, -0.845768, 3.66481, 6.14975, -1.58268, 6.26852, 6.26287, -7.41923, 4.27979, 0.687478, -11.5181, 1.79767, 0.679663, 5.31838, 7.87242, -2.22683, 9.25136, 9.80396, -1.13725, 8.53388, 7.28298, 1.40815, 5.0162, -4.05536, 1.6772, 4.1616, 7.43851, 4.6058, 0.164515, -0.319276, -0.788926, -1.14853, 5.6751, 2.42897, 6.06214, 7.07402, 1.83139, 8.43703, 4.27722, 8.11164, 9.04777, -1.58754, 9.42335, 8.04322, 4.3379, 8.67464, 5.22166, 5.49167, 7.97226, 6.09231, -3.90127, 3.00481, 4.20182, 3.06373, -0.289476, -4.36807, 4.00555, 7.30747, 7.88623, 2.9539, 6.59096, 8.01681, -1.0716, 7.0268, 4.91004, 5.90919, 7.39636, 1.40162, 8.57516, 5.35315, 6.78258, 7.70093, 0.398613, 7.5402, 2.12535, 6.48623, 6.14383, -2.29433, 2.92608, -1.35363, 2.11924, -3.27816, -8.48759, 5.79719, 6.17773, 4.68224, 9.50347, 5.56414, 9.40643, 11.193, 6.35311, 8.14396, 8.71662, 0.615323, 2.69064, -2.7849, 1.60436, 2.55466, 5.54088, -3.88065, 4.39706, -0.0289459, 3.37792, 0.0156248, 3.99434, 3.26372, 1.8326, 2.09644, 4.47424, 6.29585, -1.72902, 6.42742, -24.8898, 8.04216, 6.0148, 6.84882, 8.30132, 3.34899, 9.57759, 5.21706, 8.10474, 6.64688, 8.66846, 10.6629, -15.4439, 11.3334, 10.77, 2.8315, 10.2505, 8.06706, 1.24286, 4.50114, -3.06178, -0.160766, 4.65741, -0.170627, 5.1876, 4.55872, 4.50001, 7.63403, 2.16445, 5.5945, 4.15356, 4.18502, 6.24143, -4.02438, 4.43542, 0.567748, 3.29108, 3.90122, -2.92091, -10.4782, 1.00237, 3.26298, 1.00349, -4.33058, 3.09749, 5.5008, 5.49269, -1.24648, 4.7249, 6.0098, 3.03127, -3.3503, -1.93724, -1.49829, 7.02538, 8.5016, 1.76519, 8.56643, 9.07199, 1.52562, 10.1504, 8.90871, 4.5134, 9.2279, 5.14101, 6.71384, 7.84175, 1.34007, 3.82572, 2.47408, -0.768465, 3.92406, 4.20804, 1.22099, 1.75947, 5.85601, 4.72857, -6.57352, 1.54741, -2.57633, 1.32356, 3.17231, 7.42538, 4.80104, 5.62952, 8.21271, 4.99563, -1.04169, -5.60743, 0.958218, 0.616143, 6.35938, 1.58709, 7.6794, 8.94358, -0.919341, 8.09405, 6.81636, 3.79004, 7.11513, 2.39025, -3.5243, 2.48879, 3.61138, 4.2595, 7.99524, 1.58984, 7.95522, 6.81942, 7.38174, 10.1198, 0.931464, 10.3961, 10.2003, 2.86219, 10.1569, 7.62651, 4.78424, 5.47193, 5.00973, 7.76446, -2.92584, 7.55534, 6.78501, 1.58198, 7.49007, 6.65749, 0.841452, 1.74356, 4.27625, 3.25833, -3.9059, 5.26008, 5.9261, 1.47782, 2.15652, 4.56611, 4.15132, 3.08103, -1.19821, 2.11295, 4.94882, 1.45088, 3.13832, 4.43534, -1.6654, 6.04947, 3.79899, 5.15451, 8.58892, 7.46446, -6.93279, 6.17173, 6.64927, 2.57683, 2.7791, 6.0761, 5.3243, -0.750667, 0.552866, -0.881976, -1.12036, 0.671868, -4.8063, 2.19818, -2.25788, 3.44564, 5.36656, -2.25788, 6.71894, 8.60031, 5.14762, 6.61467, 9.3354, 6.04236, 6.44316, 8.56492, 1.84526, 6.69848, 4.71181, 6.38669, 8.63632, 3.31454, 5.70229, 4.4557, 2.33829, 1.80704, 5.12339, 5.71233, 5.6113, 9.974, 7.49701, 7.22783, 9.83519, 4.30403, 7.75844, 7.7877, -2.27489, 6.36091, 2.81898, 0.118515, -13.0225, 3.85221, 2.38218, 0.830453, 4.19751, 2.71067, 0.154469, -6.26875, 3.75924, 7.5666, 7.31697, -7.66823, 6.70891, 5.44753, 3.26892, 7.07215, 2.62678, 5.48707, 6.68678, -5.13231, 7.08438, 8.22223, 4.92026, 4.14678, 7.75647, 6.4092, -3.34271, 5.82122, 4.51702, -3.45378, 3.12326, 1.26913, 0.0827967, 3.14738, 2.32976, 2.6324, 7.162, 5.51373, 3.77164, 8.05475, 6.51418, -9.76921, 1.85002, 0.0740345, 4.60002, -2.13115, 5.40372, 6.27865, -10.2368, 6.9755, 7.44755, -1.26328, 6.26865, 5.34658, 4.20009, 8.04647, 2.90974, 8.18108, 9.54335, 2.25332, 8.51605, 8.63542, -5.51865, 7.74225, 6.39619, -8.98687, 0.228782, -1.88993, 3.86915, 6.03401, -11.0749, 8.53924, 9.35262, 1.91163, 7.12205, 5.08868, 4.48449, 4.30768, 5.38254, 7.11317, 2.62263, 9.01057, 6.19136, 6.10484, 6.76272, 3.74459, 7.36516, -0.165417, 8.71499, 6.16823, 6.10548, 7.12432, 2.6846, 7.61135, -2.87663, 7.72018, 5.78786, 6.1892, 8.64805, 3.43299, 6.29162, 7.15687, 0.593482, 5.82906, 8.59137, 8.53077, 2.12236, 8.18581, 9.75578, 4.37271, 8.08947, 8.67393, -8.19274, 6.42426, -4.81298, 7.03396, 4.39278, 6.2755, 7.65236, -5.06925, 5.45213, -4.51582, 5.79712, 2.61464, 5.08618, 5.2982, 1.62633, 5.7195, -3.40611, 3.76663, -8.78386, 5.92872, 4.28856, 2.70769, 4.32458, 2.09156, 7.04444, 5.27287, -3.52457, 2.90271, 2.63668, 4.41344, 5.14411, -7.75342, 7.83654, 9.05265, 4.55891, 6.87225, 9.03045, 6.54046, 2.00941, 6.89027, 4.54217, 4.07718, 7.6221, 3.95337, 6.63479, 8.39978, -4.17343, 8.81818, 8.23185, 3.43233, 8.30297, -2.6315, 8.77352, 7.62665, 4.90735, 7.74813, 0.87639, 8.69075, 5.00158, 7.31386, 8.03248, -5.36836, 6.12463, 0.215156, 0.833259, 4.22866, 7.81129, 0.336778, 8.8927, 9.638, 2.30358, 6.86799, 5.99062, -4.51039, -14.0763, -1.55024, 3.17275, 6.9075, 1.93539, 6.92816, 6.68417, 5.75955, 9.67067, 5.86816, 7.20585, 7.73382, 2.0439, 7.93347, 4.2095, 3.90653, 3.963, -5.66408, -1.14456, 2.50659, 0.351164, 6.80256, 4.54427, 4.56299, 5.5338, 2.64501, 5.33489, 6.21125, 10.351, 6.63846, 9.56601, 11.0485, 1.52204, 9.99343, 8.85296, 3.68785, 6.84377, 2.92282, 7.53041, -6.62546, 7.00151, -0.974457, 8.44308, 7.93271, 4.72907, 9.13737, 4.4548, 7.33835, 7.73754, -2.75439, 7.10747, 5.30834, 0.469496, 5.66952, 5.40835, 1.2916, 3.3667, 7.21888, 6.93311, 0.090553, 3.75363, 3.12093, -11.4631, -6.12691, 3.19375, 2.39046, 2.60436, 7.02564, 6.82153, 2.86594, 0.74906, 5.18327, 5.14067, -6.05443, 7.32187, 8.04129, -6.56497, 8.51772, 8.24605, 1.42621, 8.48113, 5.53294, 5.34872, 6.40955, -0.978226, 5.65988, -3.33946, 4.9873, 2.63854, 0.195223, -3.55426, 6.62963, 6.34568, 2.76784, 8.47263, 7.16078, 0.529732, 7.41542, 7.40344, 4.90075, -4.85475, 2.48812, 2.37663, -1.38693, 3.66727, -18.8641, 5.24997, 2.07235, 6.51485, 7.86573, 0.951894, 9.62429, 8.05592, 6.84077, 10.1819, 5.39706, 8.14516, 8.61271, -5.94272, 5.39461, -8.85006, 4.87281, 0.486109, 1.15049, -1.24781, 4.84443, -1.83082, 8.16731, 6.80381, 7.01186, 10.4933, 7.74399, 5.44588, 7.35365, -13.2499, 4.09952, 1.26305, 6.08523, -3.80882, 7.91474, 6.21937, 5.70371, 8.4387, 1.82076, 6.78676, 5.71478, 2.89467, 5.65577, -4.22067, 5.34488, -5.22654, 7.30321, 8.02596, 0.145819, 6.97234, 7.53535, -1.129, 6.57425, 6.98658, -4.90213, 6.25627, 4.77064, 5.58489, 9.40861, 7.89372, 3.47285, 9.15187, 8.0927, -7.10558, 6.77162, 6.18814, 0.0619396, 0.423347, 1.94529, 2.01151, 1.78993, -1.68645, -0.839002, 2.50679, 0.724031, -1.57747, 4.04259, 4.82001, 2.80596, -6.82417, -1.46596, -1.37364, -0.641936, 1.31215, -0.846131, 1.84724, 5.81959, 5.30807, -1.38322, -0.913759, -4.7673, 2.08337, -4.46101, 3.15732, 4.03391, 0.598187, -2.51824, -1.84104, 1.98917, 7.42048, 6.97382, 4.78022, 10.3175, 8.86282, 6.62229, 10.7388, 6.85729, 8.99095, 10.3982, 2.84483, 7.98117, 5.86694, 5.31831, 6.13568, 3.3602, 7.35449, -2.75939, 7.77951, 7.80722, -0.272334, 3.00516, -3.95053, 1.08149, -6.16904, 4.09122, 3.51899, -10.3631, -0.664151, -7.63345, -15.2937, 0.174861, 2.44484, -0.993075, 0.684564, 4.49765, 5.32343, 3.73607, -2.79777, 5.53548, 5.58429, -3.57534, 7.23166, 7.91251, 3.13122, 5.41932, 7.7359, 5.65299, 1.93432, 7.89152, 8.22884, 3.84375, 4.50431, 5.96443, -6.08239, 4.89638, 0.75508, 5.87702, 7.32448, -0.556138, 6.59326, 6.76215, -9.82874, 5.44862, 3.90408, -0.578881, 2.86117, -2.28511, 5.87635, 6.24304, -1.9119, 5.82768, 6.54627, -14.1831, 7.32135, 7.40182, -6.82792, 5.98247, 3.77485, 1.49079, 1.53959, 0.357882, -2.75858, 8.16302, 8.97405, -0.471787, 10.4756, 9.94976, 2.96818, 9.81674, 6.11267, 7.46419, 7.7544, 3.19845, 7.95252, 1.35916, 6.36248, 5.2223, -5.16254, -2.72055, 5.54419, 2.24394, 5.92814, 6.35145, 3.64395, 8.08245, 1.77005, 7.07048, 4.54777, 7.26464, 8.33305, 1.95535, 9.06576, 4.65643, 8.70396, 9.79997, 1.46, 8.21439, 8.40588, 4.50144, -5.11528, -1.642, 4.40981, 6.29354, 2.82998, 3.8729, 5.7406, 1.47605, -5.72814, -1.94383, 0.700988, -8.05646, -6.99731, 4.05044, 5.31722, 1.24321, 7.9939, 5.01217, 7.78623, 9.72343, 2.10711, 8.73314, 7.77586, 5.43719, 8.98829, 3.43719, 6.6263, 5.27778, 3.95228, 5.37804, 3.06888, 8.10458, 6.61401, -1.3529, 6.33744, 6.86779, 5.30456, -7.6935, 6.98252, 8.54391, 5.67698, 3.59392, 6.56881, 1.48536, 4.11307, 5.01524, 0.205157, -0.353649, 2.13126, 0.666118, -0.788108, 5.44522, 5.79351, -3.98863, 4.7727, 3.19895, 1.68219, 3.45415, -2.78432, 1.92333, 0.157271, 2.20917, 5.39791, 8.97454, 4.9906, 8.42506, 9.70936, -4.78199, 8.91736, 6.13742, 6.54109, 5.45056, 8.38135, 10.4177, -0.501907, 10.4544, 9.71138, 5.12351, 10.1397, 6.88796, 5.27403, 6.22298, -11.6571, 2.88745, 0.602895, 2.77895, 4.84503, -39.1414, 7.19376, 6.91796, 2.83215, 7.80947, 0.932365, 7.69897, 7.47791, -1.88631, 5.3442, -3.48155, -0.175797, 4.19317, 6.29161, 2.16642, 9.35874, 8.43601, 0.933334, 7.00049, -0.126666, 3.85058, -1.57629, 6.22917, -2.51451, 7.8212, 7.71341, -0.819454, 6.32018, -2.94588, 4.64178, -14.5144, 5.33665, -9.04205, 7.21153, 3.88545, 8.46596, 9.8422, -6.42219, 10.1116, 8.19354, 7.9429, 10.4545, 2.49029, 9.68478, 9.27672, 2.87727, 9.58206, 8.38671, -0.182244, 2.65743, 1.65126, 1.09877, 2.87448, 1.96258, -4.42655, 4.06806, 5.72661, 5.07904, -1.26036, 3.66922, 5.00006, -0.943129, 2.27858, 0.112103, 1.51773, 2.06852, -2.37625, -0.667541, 3.65473, 6.3374, 0.0333867, 6.12309, 6.31714, -1.69402, 6.21204, 3.83453, -1.00184, -1.27446, 1.22096, 2.48583, 0.793715, 3.66447, 5.13792, -2.7233, 8.55804, 8.93592, 0.636822, 10.3432, 9.94077, -3.6403, 8.94523, 7.20444, 1.49453, 4.92874, -4.44344, 4.24579, 0.587191, -0.719514, -2.2253, -1.99615, 1.05957, 2.46981, 2.53719, -3.81971, 5.806, 5.51493, 2.91087, 8.22318, 6.16246, 3.59808, 5.37177, 3.34867, 7.4488, -0.964381, 7.37833, 5.88982, 4.76819, 6.39296, 2.47267, 6.73755, 3.52606, 10.0552, 7.93521, 8.3156, 11.2171, 7.50811, 7.63296, 8.94544, 0.453521, 5.95288, 4.46272, -2.30121, 0.955645, 1.80382, 1.91712, 6.00402, 0.0243457, 6.27021, 6.51977, -3.30938, 4.67831, 0.754771, 7.70666, 5.55625, 5.66707, 8.317, 2.80069, 6.57991, 6.84392, -4.71769, 5.67904, 0.454736, 5.85905, 6.97847, -1.65421, 6.88484, 8.22114, 5.56589, -0.615333, 4.76293, 3.51663, -0.68161, -6.8268, -0.978784, 4.85639, 6.43217, 2.47263, 5.59198, 7.79216, 2.23856, 7.53741, 9.22844, 5.60313, 5.20076, 7.35191, 2.14982, 4.63508, 4.9989, -2.4593, 5.57293, 3.38116, 4.13501, 7.52131, 5.21141, 3.67219, 7.27679, 4.50384, 1.20586, 1.18801, 4.26619, 6.63572, -0.21046, 6.30132, 6.87629, -1.77147, 5.03394, 5.65089, 4.2542, 2.7546, -8.58195, 5.68085, 6.4992, -0.420214, 8.89407, 8.78631, -4.43113, 8.35873, 6.87797, 0.916855, 4.4501, -2.91393, 1.15921, 4.51849, 7.04079, -5.26157, 7.56895, 6.32563, 4.27707, 6.61491, 3.77836, 9.50645, 7.25242, 6.76219, 9.49737, 4.27835, 7.28569, 7.52416, -5.24377, 5.57753, -5.95003, 6.72871, 6.69664, -3.85794, 6.80261, 5.7018, 2.6581, 8.13297, 7.45468, -2.55227, 8.07263, 8.22655, 1.2008, 6.38421, 7.56354, 4.75389, -3.36335, 4.02044, 5.16571, 4.69261, 1.19467, -2.59671, -1.98465, 0.568206, 3.22361, -4.29157, 3.4973, 4.16007, -2.93162, 2.05574, 3.8025, 4.34001, 4.47397, 1.18082, 2.233, 4.9591, -0.547481, 4.29516, 5.20671, -1.97587, 2.05711, 0.443435, -8.26619, -4.95959, 2.056, 1.02152, -0.766751, 4.42674, 5.08574, 2.33246, 3.73273, 8.78936, 9.37848, 2.5236, 8.99692, 9.75702, -7.86968, 9.78537, 8.10839, 6.96804, 9.60572, -8.9212, 9.90166, 8.78327, 3.85947, 7.66777, -4.46397, 6.73275, -20.742, 7.47006, 3.02095, 7.92325, 8.08184, 3.9698, 9.15013, 5.12739, 6.53279, 6.50176, 0.662017, 4.20487, 1.33031, 3.91665, 4.59204, 8.64967, 6.2996, 3.08757, 5.61635, 0.239794, -1.77237, 4.63853, 1.83047, 6.77896, 9.03952, 4.15928, 7.16714, 7.25409, 3.14594, 8.42211, 5.46478, 5.04673, 7.46107, 4.32656, 1.29466, 5.92396, 6.25491, 0.765294, 6.32719, 8.37421, 3.43454, 7.32396, 8.17145, -3.21248, 8.18732, 6.36012, 1.72974, 1.48176, 6.05734, 7.07263, 2.82948, 9.18628, 7.2225, 5.24418, 8.45729, 4.98284, -0.760657, -5.05709, 4.441, 0.466801, 4.69129, 5.53001, -6.90546, 5.42447, 4.09392, -0.117511, 5.26764, 4.32011, -3.27103, 5.82038, 5.86778, -0.527241, 8.03577, 7.71488, 0.41352, 8.27566, 5.88878, 6.14625, 8.93884, 6.11302, 0.963935, 4.68308, 4.32149, 5.19858, 3.71418, 4.27873, 8.584, 6.83863, 2.6504, 5.22565, 3.34771, 6.86399, 2.02088, 9.36898, 6.64019, 8.23984, 9.83889, -4.04936, 9.81415, 7.30801, 7.28227, 8.45524, 2.34255, 8.52307, 2.96777, 6.33972, 1.90765, 7.25068, 6.81536, 4.139, 7.28062, -0.194996, 8.3883, 5.66986, 6.30192, 8.3664, 3.76873, 4.59239, 7.21174, 7.89924, 7.18592, -6.51145, 8.71601, 9.03575, -7.73552, 8.57051, 5.88815, 6.55297, 7.34267, 3.78877, 8.34138, -1.15824, 8.52827, 7.47124, 4.02634, 7.22795, -5.91068, 6.91996, 4.15661, 0.821709, -22.0664, 4.49897, -1.73166, 5.94517, 4.53598, 5.59399, 7.53021, -0.582211, 8.34145, 4.28926, 8.42197, 9.79725, 3.46308, 7.13826, 6.93269, 0.641337, 0.0285839, 4.81811, -0.986111, 7.18686, 8.45731, -1.45274, 8.43109, 8.65725, -1.85573, 6.98501, 5.83063, 2.66195, 7.45427, 5.75096, 1.16768, 6.13616, 1.34978, 5.96795, 7.06521, -4.2325, 7.94054, 6.58519, 5.32482, 8.95763, 5.13935, 5.91306, 5.97287, 4.08443, 7.81885, 1.98468, 6.92383, 7.77808, 3.88369, -5.7725, 0.580107, 4.90602, 7.16114, 4.22682, 6.5037, 9.33144, 6.49651, 4.46505, 6.64953, -2.57759, 4.03795, 1.44669, -9.56495, 2.36137, 4.27426, -2.29189, 6.39095, 4.20307, 4.79949, 6.99353, -0.357752, 4.59478, -0.284435, 5.40043, 5.84095, -16.1986, 4.7304, 4.41625, 2.76579, 2.31029, -4.06898, 3.03387, 3.1146, -0.228032, 2.92798, 2.73758, 6.59873, -0.1266, 10.0766, 10.0983, 2.70764, 10.9551, 9.15594, 7.41546, 10.6481, 5.89643, 7.65978, 7.36611, 1.79625, 5.63252, -1.49485, 4.2734, 3.13399, 8.01231, 4.71928, 5.96387, 7.13743, -10.3552, 5.54566, -4.48122, 6.6661, 6.45981, -4.10322, 5.62904, 4.12621, -8.31664, -1.57465, -0.624215, 3.36261, 5.55404, 4.65622, -3.476, 1.20935, 0.937935, -1.44758, 0.0296298, 0.144968, -0.420831, 5.96249, 6.6495, -0.890847, 6.78412, 7.95985, -0.831262, 8.44755, 9.55306, 3.67066, 8.22991, 9.43666, 4.16325, 6.18714, 6.11709
 };


 #endif /* MULTIPATH_V120_M10_H_ */
 
