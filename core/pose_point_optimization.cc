#include <stdlib.h>
#include <float.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <string>

#include "ceres/ceres.h"
#include "ceres/cost_function.h"
#include "ceres/rotation.h"
#include "io_pc.h"
#include "median.h"
#include <nanoflann.hpp>
using std::cin;
using std::cout;
using std::endl;
using std::string;

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> MatrixA;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix33;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 4, 1> Vector4;
typedef Eigen::Quaterniond quaternion;
typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Matrix3X;
typedef Eigen::Matrix<Scalar, 4, Eigen::Dynamic> Matrix4X;
typedef Eigen::Matrix<Scalar, 4, Eigen::Dynamic> Matrix44;
typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Vertices;

Eigen::AngleAxisd rotate_vector(M_PI * 2 / 5, Eigen::Vector3d(0, 0, 1));
Matrix33 rotate_T_72 = rotate_vector.toRotationMatrix();
Eigen::Quaterniond calulateQuaternionT(int n, int MAX_N);

#include "utils.h"
using namespace nanoflann;
PointCloud<double> cloud;

using KDtree = nanoflann::KDTreeEigenMatrixAdaptor<Matrix3X, -1, nanoflann::metric_L2_Simple, false>;

// calulateQuaternion around z axes

Eigen::Quaterniond calulateQuaternionT(int n, int MAX_N) {
    Eigen::Quaterniond q;
	// std::cout<<n<<"  "<<MAX_N<<std::endl;
    q = Eigen::AngleAxisd(M_PI * 2.0 / (double)MAX_N * (double)n, Eigen::Vector3d::UnitZ());
    return q;
};



template <typename T>
void InverseRotateMarix(const T q[4],const T t0[3*1], T R_inverse[3 * 3],T t_inverse[3 * 1]){
    using namespace ceres;
    T R[3 * 3];
    QuaternionToRotation(q, R);
    int col = 0, line = 0;
    while (col <= 2 && line <= 2) {
        R_inverse[col * 3 + line] = R[line * 3 + col];
        line++;
        if (line == 3) {
            line = 0;
            col++;
        }
    }
    for (int i = 0; i < 3; i++) {
        t_inverse[i] = R_inverse[i * 3 + 0] * t0[0] +
                      R_inverse[i * 3 + 1] * t0[1] +
                      R_inverse[i * 3 + 2] * t0[2];
        t_inverse[i] = -t_inverse[i];
    }
}



struct Cost_TP {
private:
    Matrix3X* originPoints;
    KDtree* kdtreeA;
    Eigen::Quaterniond stableRotateQ;

public:
    Cost_TP(KDtree* kdtreeA, Matrix3X* originPoints,
                Eigen::Quaterniond stableRotateQ)
        : kdtreeA(kdtreeA), originPoints(originPoints), 
        stableRotateQ(stableRotateQ){}

    template <typename T>
    bool operator()(const T* const point,
        const T* const q0,
        const T* const t0,
        T* residual) const {

        using namespace ceres;
        using namespace std;
    
        T J_q0[4];
        J_q0[0] = q0[3]; J_q0[1] = q0[0]; J_q0[2] = q0[1]; J_q0[3] = q0[2];
        // cout<<q0[0]<<" "<<q0[1]<<" "<<q0[2]<<" "<<q0[3]<<endl;
        // cout<<J_q0[0]<<" "<<J_q0[1]<<" "<<J_q0[2]<<" "<<J_q0[3]<<endl;
    
        // j_point_tp = T * P
        T j_point_tp[3];
        T jpoint[3];
        jpoint[0] = T(point[0]); jpoint[1] = T(point[1]); jpoint[2] = T(point[2]);

        UnitQuaternionRotatePoint(J_q0, jpoint, j_point_tp);//R
        j_point_tp[0] += t0[0];//t
        j_point_tp[1] += t0[1];
        j_point_tp[2] += t0[2];

       


        // j_point_stableT_tp = stable_T* T * P
        T j_point_stableT_tp[3];
 
        T qStable[4];
        auto q_temp = stableRotateQ.coeffs().data();
        qStable[0] = T(q_temp[3]);qStable[1] = T(q_temp[0]); qStable[2] = T(q_temp[1]);qStable[3] = T(q_temp[2]);
    
       
        UnitQuaternionRotatePoint(qStable, j_point_tp, j_point_stableT_tp);


        


        T j_point_inverseT_stableT_tp[3];
       
        T J_R_inverse[3 * 3], J_t_inverse[3 * 1];
        InverseRotateMarix( J_q0, t0, J_R_inverse, J_t_inverse);
        T quaternion_Inverse[4];
        RotationMatrixToQuaternion(J_R_inverse, quaternion_Inverse);
        UnitQuaternionRotatePoint(quaternion_Inverse, j_point_stableT_tp, j_point_inverseT_stableT_tp);
    
        j_point_inverseT_stableT_tp[0] += J_t_inverse[0];
        j_point_inverseT_stableT_tp[1] += J_t_inverse[1];
        j_point_inverseT_stableT_tp[2] += J_t_inverse[2];

        
        double point_inverseT_stableT_tp[3];
        point_inverseT_stableT_tp[0] = ((Jet<double, 10>)j_point_inverseT_stableT_tp[0]).a;
        point_inverseT_stableT_tp[1] = ((Jet<double, 10>)j_point_inverseT_stableT_tp[1]).a;
        point_inverseT_stableT_tp[2] = ((Jet<double, 10>)j_point_inverseT_stableT_tp[2]).a;

	
        
	    int neighbor_count =1;
  	    Eigen::Index index_kd[neighbor_count];
	    double out_distances_sq[neighbor_count];
	    kdtreeA->query(point_inverseT_stableT_tp, neighbor_count, index_kd, out_distances_sq);
	    auto point_From_kdTree = originPoints->col(index_kd[0]);
	
                                                                                                                                                                                                                                                                                                                                                                                                                     
        T j_point_From_kdTree[3];
        j_point_From_kdTree[0] = T(point_From_kdTree[0]);
        j_point_From_kdTree[1] = T(point_From_kdTree[1]);
        j_point_From_kdTree[2] = T(point_From_kdTree[2]);


        T dist[3];
        dist[0] = j_point_inverseT_stableT_tp[0] - j_point_From_kdTree[0];
        dist[1] = j_point_inverseT_stableT_tp[1] - j_point_From_kdTree[1];
        dist[2] = j_point_inverseT_stableT_tp[2] - j_point_From_kdTree[2];


        
        T distanse = T(0.0);
        distanse = sqrt(dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2]);
        if (distanse >0.03) {
          residual[0] = T(0);
          residual[1] = T(0);
          residual[2] = T(0);
        } else {
          residual[0] = dist[0];
          residual[1] = dist[1];
          residual[2] = dist[2];
        }
        // residual[0] = dist[0];
        // residual[1] = dist[1];
        // residual[2] = dist[2];

        return true;
    }

    static ceres::CostFunction* Create(KDtree* const kdtreeA,
                                    Matrix3X* originPoints,
                                    Eigen::Quaterniond q) {
    return (new ceres::AutoDiffCostFunction<Cost_TP, 3, 3, 4, 3>(
        new Cost_TP( kdtreeA, originPoints, q)));
    }
};


Matrix3X inverseVectorPoints2EigenPoints(std::vector<double*> Points){
    Matrix3X finalPoints;
    for (int i = 0; i < Points.size(); ++i) {
        Vector3 tmp;
        tmp << Points[i][0], Points[i][1], Points[i][2];
        Matrix3X tmp2(3, finalPoints.cols() + 1);
        tmp2 << finalPoints, tmp;
        finalPoints = tmp2;
    }
    return finalPoints;
}

std::vector<double*> inverseEigenPoints2VectorPoints(Matrix3X Points){
    std::vector<double*> vecPoints(Points.cols(), nullptr);
    for (int i = 0; i < Points.cols(); i++) {
        double* temp = (double*)malloc(sizeof(double) * 3);
        temp[0] = Points.col(i)[0];
        temp[1] = Points.col(i)[1];
        temp[2] = Points.col(i)[2];
        vecPoints[i] = temp;
    }
    return vecPoints;
}


double getHightMiddle(const std::vector<double*> Points){
    double Lowest = DBL_MAX;
    // double Hightest = DBL_MIN;
    double Hightest = -DBL_MAX;
    for(auto i:Points){
        if(i[2]>Hightest){
            Hightest = i[2];
        }
        if(i[2]<Lowest){
            Lowest = i[2];
        }
    }
    std::cout<<"Lowest: "<<Lowest<<" Hightest: "<<Hightest<<std::endl;
    return (Lowest+Hightest)/2.0;
}



double getHightMiddleWithT(std::vector<double*> Points, 
    Eigen::Matrix3d RR,Vector3 t){
    Matrix3X matrixpoints = inverseVectorPoints2EigenPoints(Points);
    matrixpoints = RR * matrixpoints;
    // matrixpoints = matrixpoints + t;
    std::vector<double*> temp = inverseEigenPoints2VectorPoints(matrixpoints);
    for(auto& i : temp){
        i[0] += t.data()[0];
        i[1] += t.data()[1];
        i[2] += t.data()[2];
    }
    return getHightMiddle(temp);
}




bool write2csv(string csv_path,double* R_data,double* t_data){
    using std::ofstream;
    using std::ios;
    using std::to_string;
    using std::endl;
	ofstream outFile(csv_path, ios::out);
 
	if (outFile.is_open())  
	{
        for(int i=0; i<3; i++){
            outFile<<to_string(R_data[0+i*3])<<' '
                <<to_string(R_data[1+i*3])
                <<' '<<to_string(R_data[2+i*3])
                <<' '<<to_string(t_data[i])<<endl;
        }
		
		outFile.close();
        return true;
	}else{
		std::cout << "file can't open" << std::endl;
        return false;
	}
}



int main(int argc, char** argv) {
	std::string file_source = argv[1];
    std::string str_maxN = argv[2];
	std::string str_save_path = argv[3];
    std::string inverse_pose = argv[4];

	std::istringstream ss(str_maxN);
	int MAXSpoke = 5;
	ss >> MAXSpoke;
	assert(MAXSpoke>1);
	assert(!file_source.empty());


    Matrix3X originPoints;
    Matrix3X normal_target, tar_vert_colors;
    
    Vertices vertices_source, normal_source, src_vert_colors;
   
    if(false == read_file(vertices_source, normal_source, src_vert_colors, file_source)){
        std::cout<<"fail to read pcd"<<std::endl;
        return 0;
    }
    originPoints = vertices_source;
		
    // copy points as result
    std::vector<double*> resultPoints = inverseEigenPoints2VectorPoints(originPoints);
    
    double originMiddleHeight = getHightMiddle(resultPoints);
    cout<<"originMiddleHeight"<<originMiddleHeight<<endl;

    // create kd-tree
    KDtree kdtreeA(3,originPoints);
	
    std::cout << "source: " << originPoints.rows() << "x" << originPoints.cols()
            << "!!!" << std::endl;

    ceres::Problem problem;
	
    
    // result Rotate as quaternion
    quaternion q0(1, 0, 0, 0);//w x y z
    // result t
    Vector3 t(0.0, 0.0, 0.0);
	
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1);
	// std::cout<<file_source<<std::endl;
	// std::cout<<resultPoints.size()<<std::endl;
	
    for (int i = 0; i < resultPoints.size(); ++i) {
		// std::cout<<i<<std::endl;
		for( int j=1; j<=MAXSpoke; j++){
            // std::cout<<"i="+std::to_string(i)+","<<"j="+std::to_string(j)<<std::endl;
			// std::cout<<j<<std::endl;
			ceres::CostFunction* cost_function =
            Cost_TP::Create(&kdtreeA, &originPoints, calulateQuaternionT(j,NAME_MAX));
		
			
        	problem.AddResidualBlock(cost_function,
                            loss_function,
                            resultPoints[i],
                            q0.coeffs().data(),
                            t.data());
		}
    }

   
    ceres::Solver::Options options;
    options.max_num_iterations = 1000;
   
    options.minimizer_progress_to_stdout = true;
    options.update_state_every_iteration = true;
   
    ceres::Solver::Summary summary;
	std::cout<<"summary"<<std::endl;
  
    ceres::Solve(options, &problem, &summary);
	std::cout<<"Solve"<<std::endl;

    std::cout << summary.BriefReport() << "\n" << std::endl;

    std::cout <<"IsSolutionUsable:"<< summary.IsSolutionUsable()<<std::endl;
    Eigen::Matrix3d RR;
    RR = q0.matrix();
    std::cout << "#R " << RR << "\n" << std::endl;
    std::cout << "#q " << q0 << "\n" << std::endl;
    std::cout << "#t " << t << "\n" << std::endl;

    double MiddleHeight = getHightMiddleWithT(resultPoints,RR,t);
    cout<<"MiddleHeight"<<MiddleHeight<<endl;
    t.data()[2] -=  (MiddleHeight-originMiddleHeight);
    cout << "#real_t " << t << "\n" << endl;

    std::string out_path = str_save_path;

    Matrix3X finalPoints = inverseVectorPoints2EigenPoints(resultPoints);
    

    write_file(
        file_source, finalPoints, normal_target, tar_vert_colors, out_path);
    std::cout << out_path << "\n" << std::endl;

    write2csv(inverse_pose,RR.data(),t.data());

    return 0;
}
