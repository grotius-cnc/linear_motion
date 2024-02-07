#include "linear_motion.h"
#include <cassert>

linear_motion::linear_motion()
{

}

//! Set debug state.
void linear_motion::set_debug(bool state, bool state_at_time){
    debug=state;
    debug_time_request=state_at_time;
}

//! Set curve values and calculate curve.
void linear_motion::set_curve_values(double velocity_begin,
                                     double velocity_end,
                                     double velocity_max,
                                     double acceleration_max,
                                     double displacment){
    vo=velocity_begin;
    ve=velocity_end;
    vm=velocity_max;
    a=acceleration_max;
    s=displacment;

    res={none,none,none}; // Reset.

    if(displacment>=0){
        curve_flow_positive();
    }
    if(displacment<0){
        curve_flow_negative();
    }

    print_curve_info();
}

//! Calculate total curve time.
double linear_motion::get_curve_total_time(){
    return t1+t2+t3;
}

double linear_motion::get_curve_ve(){
    if(debug){
        std::cout<<"curve ve:"<<ve<<std::endl;
    }
    return ve;
}

//! Interpolate curve at a certain time.
void linear_motion::get_curve_at_time(double t, double &sr, double &vr, double &ar){

    //! Period t1.
    if(t<=t1 && std::get<0>(res)!=3){

        if(std::get<0>(res)==0){
            // std::cout << "acc" << std::endl;
            vr=acc_dcc_ve_at_time(vo,a,t);
            sr=acc_dcc_s(vo,vr,a);
            ar=a;
        }
        if(std::get<0>(res)==1){
            //std::cout << "steady" << std::endl;
            vr=vm;
            sr=linear_lenght(vr,t);
            ar=0;
        }
        if(std::get<0>(res)==2){
            //std::cout << "dcc" << std::endl;
            vr=acc_dcc_ve_at_time(vo,-a,t);
            sr=acc_dcc_s(vo,vr,-a);
            ar=-a;
        }
    }
    //! Period t2.
    if(t>=t1 && t<=t1+t2 && std::get<1>(res)!=3){

        double ti=t-t1;
        if(std::get<1>(res)==0){
            // std::cout << "acc" << std::endl;
            vr=acc_dcc_ve_at_time(vm,a,ti);
            sr=acc_dcc_s(vm,vr,a)+s1;
            ar=a;
        }
        if(std::get<1>(res)==1){
            //std::cout << "steady" << std::endl;
            vr=vm;
            sr=linear_lenght(vr,ti)+s1;
            ar=0;
        }
        if(std::get<1>(res)==2){
            //std::cout << "dcc" << std::endl;
            vr=acc_dcc_ve_at_time(vm,-a,ti);
            sr=acc_dcc_s(vm,vr,-a)+s1;
            ar=-a;
        }
    }
    //! Period t3.
    if(t>=t1+t2 && t<=t1+t2+t3 && std::get<1>(res)!=3){

        double ti=t-t1-t2;
        if(std::get<2>(res)==0){
            // std::cout << "acc" << std::endl;
            vr=acc_dcc_ve_at_time(vm,a,ti);
            sr=acc_dcc_s(vm,vr,a)+s1+s2;
            ar=a;
        }
        if(std::get<2>(res)==1){
            //std::cout << "steady" << std::endl;
            vr=vm;
            sr=linear_lenght(vr,ti)+s1+s2;
            ar=0;
        }
        if(std::get<2>(res)==2){
            //std::cout << "dcc" << std::endl;
            vr=acc_dcc_ve_at_time(vm,-a,ti);
            sr=acc_dcc_s(vm,vr,-a)+s1+s2;
            ar=-a;
        }
    }

    if(debug_time_request){
        // Set the output to fixed-point notation
        std::cout << std::fixed << std::setprecision(3);
        std::cout<<"t:"<<t<<" s:"<<sr<<" v:"<<vr<<" a:"<<ar<<std::endl;
    }
}

//! Calculate curve periods t1,t2,t3 and their state : acc,steday,dcc,none.
//! Calculate curve time and displacement for each period t1,t2,t3.
void linear_motion::curve_flow_positive(){

    // Input validation
    if(a==0){
        std::cout<<"Error: curve construction error"<<std::endl;
        t1=0, t2=0, t3=0;
        s1=0, s2=0, s3=0;
        res=std::make_tuple(none,none,none);
        return;
    }
    if(s==0){
        std::cout<<"Error: curve construction error"<<std::endl;
        t1=0, t2=0, t3=0;
        s1=0, s2=0, s3=0;
        res=std::make_tuple(none,none,none);
    }

    s1=0;
    s2=0;
    s3=0;
    t1=0;
    t2=0;
    t3=0;

    double a1=a;
    double a2=a;
    if(vo<vm){
        a1=abs(a1);
    }
    if(vo>vm){
        a1=-abs(a1);
    }
    if(vm<ve){
        a2=abs(a2);
    }
    if(vm>ve){
        a2=-abs(a2);
    }

    s1=acc_dcc_s(vo,vm,a1);
    s3=acc_dcc_s(vm,ve,a2);
    s2=s-s1-s3;

    t1=acc_dcc_time(vo,vm,a1);
    t2=linear_time(s2,vm);
    t3=acc_dcc_time(vm,ve,a2);

    if(s2<0 && acc_dcc_s(vo,ve, a_pos_neg(vo,ve,a) )>=s && vo<ve){
        s1=s;
        s2=0;
        s3=0;
        ve=acc_ve_given_s(a,vo,s1);
        t1=acc_dcc_time(vo,ve,a);
        t2=0;
        t3=0;
        res=std::make_tuple(acc,none,none);
        return;
    }

    if(s2<0 && acc_dcc_s(vo,ve, a_pos_neg(vo,ve,a) )>=s && vo>ve){
        s1=s;
        s2=0;
        s3=0;
        ve=dcc_ve_given_s(a,vo,s1);
        t1=acc_dcc_time(vo,ve,a);
        t2=0;
        t3=0;
        res=std::make_tuple(dcc,none,none);
        return;
    }

    if(s2>=0 && vo==vm && ve==vm){
        s1=s;
        s2=0;
        s3=0;
        t1=linear_time(s1,vm);
        t2=0;
        t3=0;
        res=std::make_tuple(steady,none,none);
        return;
    }

    if(s2>=0 && vo<vm && ve<vm){
        res=std::make_tuple(acc,steady,dcc);
        return;
    }

    if(s2>=0 && vo>vm && ve<vm){
        res=std::make_tuple(dcc,steady,dcc);
        return;
    }

    if(s2>=0 && vo<vm && ve>vm){
        res=std::make_tuple(acc,steady,acc);
        return;
    }

    if(s2>=0 && vo>vm && ve>vm){
        res=std::make_tuple(dcc,steady,acc);
        return;
    }

    if(s2>=0 && vo>vm && ve==vm){
        res=std::make_tuple(dcc,steady,none);
        return;
    }

    if(s2>=0 && vo<vm && ve==vm){
        res=std::make_tuple(acc,steady,none);
        return;
    }

    if(s2>=0 && vo==vm && ve<vm){
        res=std::make_tuple(none,steady,dcc);
        return;
    }

    if(s2>=0 && vo==vm && ve>vm){
        res=std::make_tuple(none,steady,acc);
        return;
    }

    if(s2<0 && vm>vo && vm>ve){
        for(float i=vm; i>std::max(vo,ve); i-=0.1){ //! Curve vm sampled down to fit s.

            double a1=a;
            double a2=a;
            if(vo<i){
                a1=abs(a1);
            }
            if(vo>i){
                a1=-abs(a1);
            }
            if(i<ve){
                a2=abs(a2);
            }
            if(i>ve){
                a2=-abs(a2);
            }

            s1=acc_dcc_s(vo,i,a1);
            s3=acc_dcc_s(i,ve,a2);
            s2=s-s1-s3;

            if(s2>0){
                vm=i;
                t1=acc_dcc_time(vo,vm,a1);
                t2=linear_time(s2,vm);
                t3=acc_dcc_time(vm,ve,a2);
                res=std::make_tuple(acc,steady,dcc);
                return;
            }
        }
    }

    if(s2<0 && vm<vo && vm<ve){
        for(float i=vm; i<std::min(vo,ve); i+=0.1){ //! Curve vm sampled up to fit s.

            double a1=a;
            double a2=a;
            if(vo<i){
                a1=abs(a1);
            }
            if(vo>i){
                a1=-abs(a1);
            }
            if(i<ve){
                a2=abs(a2);
            }
            if(i>ve){
                a2=-abs(a2);
            }

            s1=acc_dcc_s(vo,i,a1);
            s3=acc_dcc_s(i,ve,a2);
            s2=s-s1-s3;

            if(s2>0){
                vm=i;
                t1=acc_dcc_time(vo,vm,a1);
                t2=linear_time(s2,vm);
                t3=acc_dcc_time(vm,ve,a2);
                res=std::make_tuple(dcc,steady,acc);
                return;
            }
        }
    }

    std::cout<<""<<std::endl;
    std::cout<<"Error: curve construction error"<<std::endl;
}

void linear_motion::curve_flow_negative(){

    // Input validation
    if(a==0){
        std::cout<<"Error: curve construction error"<<std::endl;
        t1=0, t2=0, t3=0;
        s1=0, s2=0, s3=0;
        res=std::make_tuple(none,none,none);
        return;
    }
    if(s==0){
        std::cout<<"Error: curve construction error"<<std::endl;
        t1=0, t2=0, t3=0;
        s1=0, s2=0, s3=0;
        res=std::make_tuple(none,none,none);
    }

    s1=0;
    s2=0;
    s3=0;
    t1=0;
    t2=0;
    t3=0;

    // Negative "s" displacment, then a negative "vm" value is valid.
    vm=-abs(vm);

    double a1=a;
    double a2=a;
    if(vo>vm){
        a1=-abs(a1);
    }
    if(vo<vm){
        a1=abs(a1);
    }
    if(vm<ve){
        a2=abs(a2);
    }
    if(vm>ve){
        a2=-abs(a2);
    }

    s1=acc_dcc_s(vo,vm,a1);
    s3=acc_dcc_s(vm,ve,a2);
    s2=s-s1-s3;

    t1=acc_dcc_time(vo,vm,a1);
    t2=linear_time(s2,vm);
    t3=acc_dcc_time(vm,ve,a2);

    if(t2<0 && abs(acc_dcc_s(vo,ve, a_pos_neg(vo,ve,a) ))>= abs(s) && vo<ve){
        s1=s;
        s2=0;
        s3=0;
        ve=acc_ve_given_s(a,vo,s1);
        t1=acc_dcc_time(vo,ve,a);
        t2=0;
        t3=0;
        res=std::make_tuple(acc,none,none);
        return;
    }

    if(t2<0 && abs(acc_dcc_s(vo,ve, a_pos_neg(vo,ve,a) ))>= abs(s) && vo>ve){
        a1=-abs(a1);
        s1=s;
        s2=0;
        s3=0;
        ve=dcc_ve_given_s(a1,vo,s1);

        t1=acc_dcc_time(vo,ve,a1);
        t2=0;
        t3=0;
        res=std::make_tuple(dcc,none,none);
        return;
    }

    if(t2>=0 && vo==vm && ve==vm){
        s1=s;
        s2=0;
        s3=0;
        t1=linear_time(s1,vm);
        t2=0;
        t3=0;
        res=std::make_tuple(steady,none,none);
        return;
    }

    if(t2>=0 && vo<vm && ve<vm){
        res=std::make_tuple(acc,steady,dcc);
        return;
    }

    if(t2>=0 && vo>vm && ve<vm){
        res=std::make_tuple(dcc,steady,dcc);
        return;
    }

    if(t2>=0 && vo<vm && ve>vm){
        res=std::make_tuple(acc,steady,acc);
        return;
    }

    if(t2>=0 && vo>vm && ve>vm){
        res=std::make_tuple(dcc,steady,acc);
        return;
    }

    if(t2>=0 && vo>vm && ve==vm){
        res=std::make_tuple(dcc,steady,none);
        return;
    }

    if(t2>=0 && vo<vm && ve==vm){
        res=std::make_tuple(acc,steady,none);
        return;
    }

    if(t2>=0 && vo==vm && ve<vm){
        res=std::make_tuple(none,steady,dcc);
        return;
    }

    if(t2>=0 && vo==vm && ve>vm){
        res=std::make_tuple(none,steady,acc);
        return;
    }

    if(t2<0 && vm>vo && vm>ve){
        for(float i=vm; i>std::max(vo,ve); i-=0.1){ //! Curve vm sampled down to fit s.

            double a1=a;
            double a2=a;
            if(vo>i){
                a1=-abs(a1);
            }
            if(vo<i){
                a1=abs(a1);
            }
            if(i<ve){
                a2=abs(a2);
            }
            if(i>ve){
                a2=-abs(a2);
            }

            s1=acc_dcc_s(vo,i,a1);
            s3=acc_dcc_s(i,ve,a2);
            s2=s-s1-s3;

            if(s2<0){
                vm=i;
                t1=acc_dcc_time(vo,vm,a1);
                t2=linear_time(s2,vm);
                t3=acc_dcc_time(vm,ve,a2);
                res=std::make_tuple(acc,steady,dcc);
                return;
            }
        }
    }

    if(t2<0 && vm<vo && vm<ve){
        for(float i=vm; i<std::min(vo,ve); i+=0.1){ //! Curve vm sampled up to fit s.

            double a1=a;
            double a2=a;
            if(vo>i){
                a1=-abs(a1);
            }
            if(vo<i){
                a1=abs(a1);
            }
            if(i<ve){
                a2=abs(a2);
            }
            if(i>ve){
                a2=-abs(a2);
            }

            s1=acc_dcc_s(vo,i,a1);
            s3=acc_dcc_s(i,ve,a2);
            s2=s-s1-s3;

            if(s2<0){
                vm=i;
                t1=acc_dcc_time(vo,vm,a1);
                t2=linear_time(s2,vm);
                t3=acc_dcc_time(vm,ve,a2);
                res=std::make_tuple(dcc,steady,acc);
                return;
            }
        }
    }

    std::cout<<""<<std::endl;
    std::cout<<"Error: curve construction error"<<std::endl;
}

void linear_motion::print_curve_info(){

    if(debug){
        std::cout << std::fixed << std::setprecision(3);

        std::cout<<""<<std::endl;
        std::cout<<"- Curve input -"<<std::endl;
        std::cout<<"vo:"<<vo<<" vm:"<<vm<<" ve:"<<ve<<" s:"<<s<<" a:"<<a<<std::endl;
        std::cout<<""<<std::endl;

        std::cout<<"- Curve period t1 :";
        if(std::get<0>(res)==0){
            std::cout << "acc" << std::endl;
        }
        if(std::get<0>(res)==1){
            std::cout << "steady" << std::endl;
        }
        if(std::get<0>(res)==2){
            std::cout << "dcc" << std::endl;
        }
        if(std::get<0>(res)==3){
            std::cout << "none" << std::endl;
        }

        std::cout<<"- Curve period t2 :";
        if(std::get<1>(res)==0){
            std::cout << "acc" << std::endl;
        }
        if(std::get<1>(res)==1){
            std::cout << "steady" << std::endl;
        }
        if(std::get<1>(res)==2){
            std::cout << "dcc" << std::endl;
        }
        if(std::get<1>(res)==3){
            std::cout << "none" << std::endl;
        }

        std::cout<<"- Curve period t3 :";
        if(std::get<2>(res)==0){
            std::cout << "acc" << std::endl;
        }
        if(std::get<2>(res)==1){
            std::cout << "steady" << std::endl;
        }
        if(std::get<2>(res)==2){
            std::cout << "dcc" << std::endl;
        }
        if(std::get<2>(res)==3){
            std::cout << "none" << std::endl;
        }
        std::cout<<""<<std::endl;

        std::cout<<"- Curve times :"<<std::endl;
        std::cout<<"t1:"<<t1<<std::endl;
        std::cout<<"t2:"<<t2<<std::endl;
        std::cout<<"t3:"<<t3<<std::endl;
        ttot=t1+t2+t3;
        std::cout<<"ttot:"<<t1+t2+t3<<std::endl;
        std::cout<<""<<std::endl;

        std::cout<<"- Curve displacement :"<<std::endl;
        std::cout<<"s1:"<<s1<<std::endl;
        std::cout<<"s2:"<<s2<<std::endl;
        std::cout<<"s3:"<<s3<<std::endl;
        stot=s1+s2+s3;

        std::cout<<"stot:"<<s1+s2+s3<<std::endl;
        std::cout<<""<<std::endl;
    }
}

// Here, "s" represents the position, "ve" represents the velocity, "vo" is the initial velocity, and "t" is the time.
// "a" represents acceleration, "d" represents deceleration.

// Linear acceleration:
// s = vo * t + 0.5 * a * t*t
// ve = vo + a * t

// Linear deceleration:
// s = u * t − 0.5 * ​d * t*t
// ve = vo − d * t

// Steady:
// s = vo * t
// ve = vo

// Calculate stopping distance during deceleration until velocity becomes zero
double linear_motion::stop_length(double v, double a){
    // Stopping distance during deceleration: s = (ve^2 - vo^2) / (2 * |a|)
    return (v * v) / (2 * std::abs(a));
}

double linear_motion::acc_dcc_time(double vo, double ve, double a){
    // Formula: time = (ve - vo) / a
    // Avoid division by zero
    // Time output must always be positive, therefore abs(a) is a<0
    if (a == 0.0) {
        return 0.0;  // Acceleration is zero, time is not well-defined
    }
    // Calculate acceleration time
    return abs(ve - vo) / abs(a);
}

double linear_motion::acc_dcc_s(double vo, double ve, double a){

    // std::cout<<"vo:"<<vo<<std::endl;
    // std::cout<<"ve:"<<ve<<std::endl;
    //std::cout<<"a:"<<a<<std::endl;

    // Acceleration length: s = (ve^2 - vo^2) / (2 * a)
    double sr=(ve * ve - vo * vo) / (2 * a);

    //std::cout<<"result s:"<<sr<<std::endl;

    return sr;
}

double linear_motion::acc_dcc_ve_at_time(double vo, double a, double t) {
    // Calculate final velocity given time t.
    return vo + a * t;
}

double linear_motion::acc_ve_given_s(double a, double vo, double s) {
    // Kinematic equation: ve = sqrt(vo^2 + 2 * a * s)
    return sqrt((vo * vo) + 2 * a * s);
}

double linear_motion::dcc_ve_given_s(double a, double vo, double s) {
    // Kinematic equation: ve = sqrt(vo^2 + 2 * a * s)
    //return sqrt((vo * vo) - 2 * a * s);

    // Kinematic equation: ve = sqrt(vo^2 + 2 * a * s)
    double ve_result = vo * vo + 2 * a * s;
    if (ve_result < 0) {
        // Handle cases where the result is negative
        return -sqrt(-ve_result);
    } else {
        return -sqrt(ve_result);
    }
}

double linear_motion::linear_lenght(double v, double t){
    // Linear motion distance: s = v * t
    return v * t;
}

double linear_motion::linear_time(double s, double v){
    // Formula: time = distance / velocity
    // Avoid division by zero
    if (v == 0.0) {
        return 0;  // Velocity is zero, time is not well-defined
    }
    // Calculate linear motion time
    return s / v;
}

double linear_motion::a_pos_neg(double vo, double ve, double a){
    if(vo<ve){
        a=abs(a);
    }
    if(vo>ve){
        a=-abs(a);
    }
    return a;
}
