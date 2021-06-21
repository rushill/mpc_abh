#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

ros::Publisher pub_vel;
geometry_msgs::Twist pub_twist;
turtlesim::Pose pos1, pos2;

void t1_callback(const turtlesim::Pose::ConstPtr& _pos1){

    pos1 = *_pos1;
    // assign the value of turtle1's pose to pos1
}

void t2_callback(const turtlesim::Pose::ConstPtr& _pos2){

    pos2 = *_pos2;
    // assign the value of turtle1's pose to pos1
}

void MPC(float , float);
size_t N = 10;
double dt = 0.1;

int main(int argc, char* argv[]){
   



    ros::init(argc, argv, "mpc");
    float x, y;
    std::cout<<"a";

    ros::NodeHandle t_sim;
   

    ros::Rate rate(2);


    pub_vel = t_sim.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 1000);


    ros::Subscriber t1 = t_sim.subscribe("/turtle1/pose", 1000, t1_callback);


    ros::Subscriber t2 = t_sim.subscribe("/turtle2/pose", 1000, t2_callback);


    rate.sleep();


    ros::spin();

    char *again;
    do{
        std::cout<<"Enter X:\n";
        std::cin>>x;
        std::cout<<"\nEnter Y:\n";
        std::cin>>y;
        MPC(x, y);
        std::cout<<"\nAgain? (y/n) \n";
        std::cin>>again;
    }while(again=="y");
}

void MPC(float goal_x, float goal_y){
    pub_twist.linear.x = 0;
    pub_twist.linear.y = 0;
    pub_twist.linear.z = 0;

    pub_twist.angular.x = 0;
    pub_twist.angular.y = 0;
    pub_twist.angular.z = 0;
    float velocity = 0;


    size_t v_start = 0;
    size_t w_start = v_start + N -1;
    size_t x_start = 0;
    size_t y_start = x_start + N;
    size_t t_start = y_start + N - 1;

    class FG_eval {
    public:
        float goal_x, goal_y;
        FG_eval(float x, float y){
            goal_x=x;
            goal_y=y;
        }
        typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;
        void operator()(ADvector& fg, const ADvector& x)
        {   
            size_t v_start = 0;
            size_t w_start = v_start + N -1;
            size_t x_start = 0;
            size_t y_start = x_start + N;
            size_t t_start = y_start + N - 1;
            ADvector pose(3*N);
            pose[x_start] = pos1.x;
            pose[y_start] = pos1.y;
            pose[t_start] = pos1.theta;
            fg[0]=0; 
            for (size_t i=0; i<N;++i){

                pose[x_start+i]=pose[x_start+i-1]+x[v_start+i-1]*CppAD::cos(pose[t_start+i-1])*dt;
                pose[y_start+i]=pose[y_start+i-1]+x[v_start+i-1]*CppAD::sin(pose[t_start+i-1])*dt;
                pose[t_start+i]=pose[t_start+i-1]+x[w_start+i-1]*dt;

                fg[0] += CppAD::pow((goal_x-pose[x_start+i]), 2);
                fg[0] += CppAD::pow((goal_y-pose[y_start+i]), 2);

                fg[1+i] = CppAD::pow((pos1.x-pose[x_start+i]), 2) + CppAD::pow((pos1.y-pose[y_start+i]), 2);

            }
            return;
        }
     };
    
    typedef CPPAD_TESTVECTOR( double ) Dvector;
    size_t nx=2*N;
    size_t ng=N;

    Dvector xi(nx);
    for(size_t i = 0; i<nx; ++i){
        xi[i]=velocity;
    }

    Dvector xl(nx), xu(nx);
    for(size_t i=0; i<nx; ++i){
        if(i<N){
            xl[i]=((velocity-0.1)<-2) ? -2 : (velocity - 0.1); xu[i]=((velocity+0.1)>2) ? 2 : (velocity + 0.1);
        }
        else{
            xl[i]=-0.2, xu[i]=0.2;
        }
    }

    Dvector gl(ng), gu(ng);
    for (size_t i = 0; i<ng;++i){
        gl[i]=0;
        gu[i]=0.25;

    }

    FG_eval fg_eval(goal_x, goal_y);
    std::string options;

    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";

    // Disables printing IPOPT creator banner
    options += "String  sb          yes\n";

    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";

    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    CppAD::ipopt::solve_result<Dvector> solution;

    do{
        CppAD::ipopt::solve<Dvector, FG_eval>(
        options, xi, xl, xu, gl, gu, fg_eval, solution);
        pub_twist.linear.x = solution.x[0];
        pub_twist.angular.z = solution.x[N];
        pub_vel.publish(pub_twist);

    }while(solution.obj_value>=0.01);

}
 
