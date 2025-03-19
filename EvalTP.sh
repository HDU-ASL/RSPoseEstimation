array_scene=(
    螺旋桨/"PropellerBoat_A.1_(step)"
    螺旋桨/propeller-for-boat-and-ship-1_2_STP_IGS_CATIA
    螺旋桨/propeller-284_3_SW_STL#3
    叶轮/part2
    叶轮/wirnik2
    轮毂/15_x_6.5_Gravel_Rally_Wheel
    轮毂/16x7_Enkei_J10
    轮毂/Alloy_Rim
    轮毂/aston_martin
    零件/AGF6-25R1_On
    零件/AGF6-50R1_On
    零件/Bearing,GB-T301_53000-1995
    零件/Nut,GB-T41-2000
    零件/nut1
)
for scene_str in "${array_scene[@]}"
do
    scene=logs/$scene_str/
    # echo $scene
    # python eval.py --Points=$scene/TruePose/TruePoints.ply --predictPose=$scene/PredictPoseAndPose/PredictPose.csv --truePose=$scene/TruePose/TruePose.csv
    # python eval.py --Points=$scene/TruePose/TruePoints.ply --predictPose=$scene/PredictPoseAndPose_1V/PredictPose.csv --truePose=$scene/TruePose/TruePose.csv
    # python eval.py --Points=$scene/TruePose/TruePoints.ply --predictPose=$scene/PredictPoseAndPose_2V/PredictPose.csv --truePose=$scene/TruePose/TruePose.csv
    # python eval.py --Points=$scene/TruePose/TruePoints.ply --predictPose=$scene/PredictPoseAndPose_3V/PredictPose.csv --truePose=$scene/TruePose/TruePose.csv
    # python eval.py --Points=$scene/TruePose/TruePoints.ply --predictPose=$scene/PredictPoseAndPose_4V/PredictPose.csv --truePose=$scene/TruePose/TruePose.csv
    # python eval.py --Points=$scene/TruePose/TruePoints.ply --predictPose=$scene/PredictPoseAndPose_5V/PredictPose.csv --truePose=$scene/TruePose/TruePose.csv
    # python eval.py --Points=$scene/TruePose/TruePoints.ply --predictPose=$scene/PredictPoseAndPose_0.001/PredictPose.csv --truePose=$scene/TruePose/TruePose.csv
    # python eval.py --Points=$scene/TruePose/TruePoints.ply --predictPose=$scene/PredictPoseAndPose_0.003/PredictPose.csv --truePose=$scene/TruePose/TruePose.csv
    # python eval.py --Points=$scene/TruePose/TruePoints.ply --predictPose=$scene/PredictPoseAndPose_0.004/PredictPose.csv --truePose=$scene/TruePose/TruePose.csv
    python eval.py --Points=$scene/TruePose/TruePoints.ply --predictPose=$scene/PredictPoseAndPose_0.005/PredictPose.csv --truePose=$scene/TruePose/TruePose.csv
    
    
done
