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

number_array=(
    4
    3
    3
    20
    2
    5
    5
    5
    5
    25
    50
    11
    6
    20
)

exeT=ceres-solver/build/bin/t
exeTP=ceres-solver/build/bin/tp

length=${#array_scene[@]}

for ((i=0;i<$length;i++));do
    scene=${array_scene[$i]}
    number=${number_array[$i]}
    
    echo $scene
    
    depthpath=Datasets/$scene/720p_b/depth
    outputfolder=logs/$scene/PredictPoseAndPose
    camerapath=Datasets/$scene/camera_transforms.txt

    generatePointPath=$outputfolder/CreatedPoints.ply
    PointsWithoutBottom=$outputfolder/PointsWithoutBottom.ply
    PointsFilter=$outputfolder/FilterPoints.ply
    cursePosePoints=$outputfolder/cursePosePoints.ply
    cursePose=$outputfolder/T_curse.csv

    mkdir -p $outputfolder

    precisePosePoints=$outputfolder/precisePosePoints.ply
    precisePose=$outputfolder/precisePose.csv
    mixPose=$outputfolder/mixPose.csv

    python generatePointCloudFromDepth.py --path_dir=$depthpath --camera_path=$camerapath --save_path=$generatePointPath

    python DeleteBottom.py --load_path=$generatePointPath --save_path=$PointsWithoutBottom --height_bottom=0.084

    python PointFilter.py --load_path=$PointsWithoutBottom --save_path=$PointsFilter

    python CursePoseOptimize.py --load_path=$PointsFilter --save_path=$cursePosePoints --save_matrix=$cursePose

    

    
    $exeTP $cursePosePoints $number $precisePosePoints $precisePose
    python MixPose.py --cursePose=$cursePose --precisePose=$precisePose --mixPose=$mixPose


    python calMax.py --pose_T=$mixPose --pc=$precisePosePoints --pic=$outputfolder/saved.png --FinalPoints=$outputfolder/PredictPoints.ply --FinalPose=$outputfolder/PredictPose.csv --Number=$number
done

