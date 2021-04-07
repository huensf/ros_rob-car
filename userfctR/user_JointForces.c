//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h" 

#include "mbs_data.h"
#include "user_model.h"
#include "set_output.h"
#include "useful_functions.h"

#include "user_all_id.h"
#include "user_model.h"



double* user_JointForces(MbsData *mbs_data, double tsim)
{
/*-- Begin of user code --*/

    if (tsim == 0.0) // equilibrium process and modal analysis
    {
        mbs_data->Qq[R2_wheel_rr_lt_id] = mbs_data->user_model->EquilQuantities.Qpropulsion;
        mbs_data->Qq[R2_wheel_rr_rt_id] = mbs_data->user_model->EquilQuantities.Qpropulsion;
        mbs_data->Qq[T2_rack_id] = mbs_data->user_model->EquilQuantities.Qrack;
    }

    if(mbs_data->process == 2)
    {
        int pedals_mode = mbs_data->user_model->pedals.mode;

        // PEDALS ON : Acceleration and braking
        if(pedals_mode == 1) {
            double ped1 = mbs_data->user_model->pedals.ped1;
            double ped2 = mbs_data->user_model->pedals.ped2;
            double ped1_rest = 7000;
            double ped2_rest = 400;
            double speed = 3.6 * sqrt(pow(mbs_data->qd[T1_chassis_id],2)+pow(mbs_data->qd[T2_chassis_id],2));
           
            if(ped1 < ped1_rest && ped2 >= ped2_rest && speed > 10) {
                // Pedale de frein enfoncee & vitesse > 10 km/h
                mbs_data->Qq[R2_wheel_rr_lt_id] = -665*0.4;
                mbs_data->Qq[R2_wheel_rr_rt_id] = -665*0.4;
                mbs_data->Qq[R2_wheel_ft_lt_id] = -1235*0.4;
                mbs_data->Qq[R2_wheel_ft_rt_id] = -1235*0.4;
            }
            else if(ped1 >= ped1_rest && ped2 < ped2_rest && speed < 120) {
                // Pedale d'acceleration enfoncee & vitesse < 120 km/h
                mbs_data->Qq[R2_wheel_rr_lt_id] = 100*0.4;
                mbs_data->Qq[R2_wheel_rr_rt_id] = 100*0.4;
                mbs_data->Qq[R2_wheel_ft_lt_id] = 200*0.4;
                mbs_data->Qq[R2_wheel_ft_rt_id] = 200*0.4;
            }
            else {
                // Aucune pedale enfoncee (ou les deux) --> on ne fait rien
            }
        }

        // PEDALS OFF : PID Controller
        else if(pedals_mode == 2) {
            double error = mbs_data->user_model->PID.velocity - (sqrt(pow(mbs_data->qd[1],2)+pow(mbs_data->qd[2],2)));
          
            if(error >= -0.1 && error <= 0.1) mbs_data->user_model->PID.sum_error = 0;
          
            mbs_data->user_model->PID.sum_error += error;
          
            double error_variation = error-mbs_data->user_model->PID.previous_error;

            mbs_data->user_model->PID.previous_error = error;

            double commande  = mbs_data->user_model->PID.Kp*error + mbs_data->user_model->PID.Ki*mbs_data->user_model->PID.sum_error + mbs_data->user_model->PID.Kd*error_variation;

            if(commande > 750)
            {
                commande = 750;
            }

            if(mbs_data->user_model->PID.brake == 1 && sqrt(pow(mbs_data->qd[1],2)+pow(mbs_data->qd[2],2)) > 25/3.6) //brake with space barre
            {
                mbs_data->Qq[R2_wheel_rr_lt_id] = -665*0.4;
                mbs_data->Qq[R2_wheel_rr_rt_id] = -665*0.4;
                mbs_data->Qq[R2_wheel_ft_lt_id] = -1235*0.4;
                mbs_data->Qq[R2_wheel_ft_rt_id] = -1235*0.4;
            }
            else
            {
                mbs_data->Qq[R2_wheel_rr_lt_id] = mbs_data->user_model->EquilQuantities.Qpropulsion + commande;
                mbs_data->Qq[R2_wheel_rr_rt_id] = mbs_data->user_model->EquilQuantities.Qpropulsion + commande;
                mbs_data->Qq[R2_wheel_ft_lt_id] = 0;
                mbs_data->Qq[R2_wheel_ft_rt_id] = 0;
            }
        }

    }

    // ANTI-ROLL BAR
    mbs_data->Qq[R2_def_bar_ft_id] = -mbs_data->user_model->FrontSuspension.C_bar*mbs_data->q[R2_def_bar_ft_id]*2;
    mbs_data->Qq[R2_def_bar_rr_id] = -mbs_data->user_model->RearSuspension.C_bar*mbs_data->q[R2_def_bar_rr_id];


    return mbs_data->Qq;
}
