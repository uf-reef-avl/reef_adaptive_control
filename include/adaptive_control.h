//
// Created by jonah on 7/10/19.
//

#ifndef ADAPTIVECTRL_ADAPTIVE_CONTROL_H
#define ADAPTIVECTRL_ADAPTIVE_CONTROL_H

#include <eigen3/Eigen/Dense>
#include <iostream>

#include "state_space.h"

using namespace std;

// Definition of adaptive Controller class
class AdaptiveController
{
public:



    // The following constructors and methods are used for initialization
    // and setting controller parameters to the desired values
    // ----------------------------------------------------------------------

    // Default Constructor
    AdaptiveController() = default;

    // Constructor with TF
    AdaptiveController(TransferFunction TF)
    {
        // Initialize the controller
        InitializeControl(TF);
    }

    // Method to initialize controller with a transfer function model
    // This (could?) read from a parameter file, but accepts a TF argument for now
    bool InitializeControl(TransferFunction ModelTF)
    {
        // Matrix and vector args to pass to state space model
        // The first half are for the model, the second half are for
        // the filters on input and output measurements
        Eigen::MatrixXd Am;
        Eigen::VectorXd bm;
        Eigen::VectorXd cm;

        Eigen::MatrixXd Af;
        Eigen::MatrixXd Bf;
        Eigen::MatrixXd Cf;

        // Set model order
        int ModelOrder = ModelTF.GetOrder();

        // Set sizes of args, initialized all elements to zero
        // again, these are for the model and filters
        Am.setZero(ModelOrder, ModelOrder);
        bm.setZero(ModelOrder);
        cm.setZero(ModelOrder);

        // set filter order
        int FilterOrder = ModelOrder - 1;

        Af.setZero(FilterOrder, FilterOrder);
        Bf.setZero(FilterOrder, 1);
        Cf.setZero(FilterOrder, FilterOrder);


        // Set Am matrix to a superdiagonal with ones
        // and denominator coefficients on bottom row
        // Also set cm vector to coefficients of numerator
        for (int i = 0; i < ModelOrder; i++)
        {
            if (i + 1 < ModelOrder)
                Am(i, i + 1) = 1;
            Am(ModelOrder - 1, i) = -ModelTF.dens(i);
            cm(i) = ModelTF.nums(i);
        }

        // Set bm vector to [0 0 0 0 ... 1]
        bm(ModelOrder - 1) = 1;

        // Now, set the filter state space matrices
        // The filter transfer function is:
        // [1 s s^2 s^3 ... s^n] / (model TF numerator)
        // where n is one less than the highest degree in the numerator
        // This system has one input and n + 1 outputs

        // Set Af matrix to a superdiagonal with ones
        // and numerator coefficients on bottom row
        // Also set cf matrix
        for (int i = 0; i < FilterOrder; i++)
        {
            if (i + 1 < FilterOrder)
                Af(i, i + 1) = 1;
            // make sure first element is one, so everything is normalized here
            Af(FilterOrder - 1, i) = -ModelTF.nums(i) / ModelTF.nums(FilterOrder);
            Cf.setIdentity();
        }

        bool success = true;

        // Now create a state space representation from this
        success = success && SetModel(Am, bm, cm);

        // Create filters
        success = success && SetFilters(Af, Bf, Cf);

        // Sets regression vector size: two filtered terms, with Yp, r, and 1
        int regressionVectorSize = 2 * Cf.cols() + 3;

        regressionVector.setZero(regressionVectorSize);
        theta.setZero(regressionVectorSize);

        // Set Yp/Up vector
        Yp.setZero(1);
        Up.setZero(1);

        if (!success)
        {
            cerr << "Warning: Adaptive controller was not initialized." << endl;
            cerr << "Something went wrong setting state space matrices." << endl;
        }


        // Set initialization to true if successful
        isInitialized = success;
        return isInitialized;
    }

    // Enables adaptation. Must be called for controller to work
    bool EnableAdaptation()
    {
        // Begin running adaptive controller if the system
        // has been initialized to the proper values
        // Returns false if initialization failed
        enableAdaptation = isInitialized;
        return enableAdaptation;
    }

    // Method for disabling adaptation
    // This resets parameter estimates, and forces adaptive torque to return 0
    // this can be reverted by calling the EnableAdaptation() method
    void DisableAndResetAdaptation()
    {
        enableAdaptation = false;
        theta.setZero(regressionVector.size());
        u_ad = 0;
    }

    // Setter for adaptive gain
    void SetAdaptiveGain(double gamma)
    {
        adaptiveGain = gamma;
    }



    // These public methods are used to update the regression vector with new values
    // Input args are scalar, but some of the values go into a one-dimensional vector
    // Note: delta time should be computed and passed here as argument
    // -----------------------------------------------------------------------------

    void UpdatePlantOutput(double Yp_update, double delta_t)
    {
        if (isInitialized)
        {
            Yp(0) = Yp_update;
            outputFilter.Propagate(Yp, delta_t);
        }

    }

    void UpdateControlInput(double Up_update, double delta_t)
    {
        if (isInitialized)
        {
            Up(0) = Up_update;
            inputFilter.Propagate(Up, delta_t);
        }
    }

    void UpdateReferenceInput(double r_update, double delta_t)
    {
        r = r_update;
        referenceModel.Propagate(r, delta_t);
    }



    // Calling this method adapts the controller's parameters
    // This is performed using the current regression vector
    // -------------------------------------------------------------------------

    void AdaptParameters(double delta_time)
    {
        if (enableAdaptation)
        {
            UpdateError();
            UpdateRegVector();
            theta = theta + (adaptiveGain * error * regressionVector) * delta_time;
        }
    }



    // This computes and returns the adaptive control input to the plant
    // Note: returns zero if adaptation is disabled
    //--------------------------------------------------------------------------

    double GetAdaptiveTorque()
    {
            if (enableAdaptation)
            {
                u_ad = regressionVector.dot(theta);

                // Apply saturation to limit adaptive output to +/- 0.5
                if (u_ad > 0.5)
                    u_ad = 0.5;
                if (u_ad < -0.5)
                    u_ad = -0.5;
            }
            return u_ad;
    }




    // This is a getter method for the model error
    // Intended for debugging purposes only
    //--------------------------------------------------------------------------

    double GetModelError()
    {
        return error;
    }



    // The following are private variables held by the controller
    // --------------------------------------------------------------------------

private:
    // State space systems for propagating diff EQs
    StateSpace outputFilter;
    StateSpace inputFilter;
    StateSpaceSISO referenceModel;

    bool enableAdaptation = false;
    bool isInitialized = false;
    double adaptiveGain = 0.001;

    // Vector for plant output
    Eigen::VectorXd Yp;

    // Vector for plant input
    Eigen::VectorXd Up;

    // Scalar for reference input
    double r = 0;

    // scalar for model
    double Ym = 0;

    // Vector for error
    double error = 0;

    // Vectors for adaptive control law computation
    Eigen::VectorXd regressionVector;
    Eigen::VectorXd theta;

    // U for adaptive control
    double u_ad = 0;



    // The following methods are private methods for updating things
    // --------------------------------------------------------------------

    void UpdateError()
    {
        // Assumes SISO case, so error is scalar
        // Only first value of Yp is used
        Ym = referenceModel.getOutput();
        error = Ym - Yp(0);
    }

    // Updates the regression vector for computing control law and
    // parameter estimate change, theta dot
    void UpdateRegVector()
    {
        regressionVector <<     Yp,
                                outputFilter.getOutput(),
                                inputFilter.getOutput(),
                                r,
                                1;
    }



    // The following methods are used to generate the state space models
    // Arguments are the matrices A, B, and C
    // If procedure fails, false is returned
    //------------------------------------------------------------------------------

    bool SetFilters(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C)
    {
        // Default state is successful. If any actions fail, false is returned
        bool success = true;

        // Set sizes of state space system for input and output filters
        inputFilter.resize(A.rows(), B.cols(), C.rows());
        outputFilter.resize(A.rows(), B.cols(), C.rows());

        success = success && inputFilter.setA(A);
        success = success && outputFilter.setA(A);

        success = success && inputFilter.setB(B);
        success = success && outputFilter.setB(B);

        success = success && inputFilter.setC(C);
        success = success && outputFilter.setC(C);

        return success;
    }

    bool SetModel(Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::VectorXd C)
    {
        // Default state is successful. If any actions fail, false is returned
        bool success = true;

        // Size SISO model to desired condition
        referenceModel.resize(A.rows());

        success = success && referenceModel.setA(A);
        success = success && referenceModel.setB(B);
        success = success && referenceModel.setC(C);

        return success;
    }
};

#endif //ADAPTIVECTRL_ADAPTIVE_CONTROL_H
