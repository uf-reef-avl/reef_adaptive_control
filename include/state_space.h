//
// Created by jonah on 7/10/19.
// Intended for use with adaptive controller
//

#include <eigen3/Eigen/Dense>
#include <iostream>

#ifndef ADAPTIVECTRL_STATE_SPACE_H
#define ADAPTIVECTRL_STATE_SPACE_H

// Define state space class
// This is a differential equation of the form:
// [x_dot] = [A][x] + [b][u]
// [y] = [c][x]
// u is input vector, y is output vector

using namespace std;

class TransferFunction
{
public:
    TransferFunction() = default;

    void SetCoefficients(Eigen::VectorXd numerator, Eigen::VectorXd denominator)
    {
        if (numerator.size() > denominator.size())
        {
            cerr << "Transfer function is not proper. Terms not set." << endl;
        }
        else if (numerator.size() == denominator.size())
        {
            cerr << "Transfer function is not strictly proper. Terms not set." << endl;
        }
        else
        {
            num = numerator;
            den = denominator;

            order = den.size() - 1;

            num = num / den(order);
            den = den / den(order);
        }
    }

    double dens(int i)
    {
        if (i > (den.size() + 1))
            return 0;
        else if (i < 0)
            return 0;
        else if (order != -1)
            return den(i);
    }

    double nums(int i)
    {
        if (i > (num.size() + 1))
            return 0;
        else if (i < 0)
            return 0;
        else if (order != -1)
            return num(i);
    }

    TransferFunction(Eigen::VectorXd numerator, Eigen::VectorXd denominator)
    {
        SetCoefficients(numerator, denominator);
    }

    int GetOrder()
    {
        return order;
    }

private:
    Eigen::VectorXd num;
    Eigen::VectorXd den;
    int order = -1;
};

class StateSpace
{
public:
    void resize(int A_size, int num_inputs, int num_outputs)
    {
        A_dim = A_size;
        outputs = num_outputs;
        inputs = num_inputs;

        A.setZero(A_dim, A_dim);
        B.setZero(A_dim, inputs);
        C.setZero(outputs, A_dim);

        u.setZero(inputs);
        y.setZero(outputs);
        x.setZero(A_dim);
    }

    // Empty constructor
    StateSpace()
    {
        resize(0, 0, 0);
    }

    // Creates matrices of specified sizes, and initializes values to zero
    StateSpace(int A_size, int outputs, int inputs) : outputs(outputs), inputs(inputs), A_dim(A_size)
    {
         A.setZero(A_dim, A_dim);
         B.setZero(A_dim, inputs);
         C.setZero(outputs, A_dim);

         u.setZero(inputs);
         y.setZero(outputs);
         x.setZero(A_dim);
    }

    // If dimensions of argument and A are equal, set matrix A
    // Error code of -1 is returned if dimensions are incorrect
    bool setA(Eigen::MatrixXd M)
    {
        if (M.rows() != A.rows())
        {
            return false;
        }
        if (M.cols() != A.cols())
        {
            return false;
        }
        A = M;
        return true;
    }

    // If dimensions of argument and B are equal, set matrix B
    // Error code of -1 is returned if dimensions are incorrect
    bool setB(Eigen::MatrixXd M)
    {
        if (M.rows() != B.rows())
        {
            return false;
        }
        if (M.cols() != B.cols())
        {
            return false;
        }
        B = M;
        return true;
    }

    // If dimensions of argument and C are equal, set matrix C
    // Error code of -1 is returned if dimensions are incorrect
    bool setC(Eigen::MatrixXd M)
    {
        if (M.rows() != C.rows())
        {
            return false;
        }
        if (M.cols() != C.cols())
        {
            return false;
        }
        C = M;
        return true;
    }

    // Method for propagating the differential equation through time
    int Propagate(Eigen::VectorXd new_u, double dt)
    {
        u = new_u;
        x = x + (A * x + B * u) * dt;
    }

    // Computes and returns output y
    Eigen::VectorXd getOutput()
    {
        y = C * x;
        return y;
    }

    void HelpDebug()
    {
        cout << "Matrix A: " << endl << A << endl;
        cout << "Matrix B: " << endl << B << endl;
        cout << "Matrix C: " << endl << C << endl;
        cout << "Vector x: " << endl << x << endl;
        cout << "Vector y: " << endl << y << endl;
    }

private:
    int A_dim;
    int inputs;
    int outputs;

    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;

    Eigen::VectorXd u;
    Eigen::VectorXd y;
    Eigen::VectorXd x;
};

// Define state space class
// This is a differential equation of the form:
// [x_dot] = [A][x] + [b]u
// y = [c][x]
// u is a scalar input and y is a scalar output
// b and c are vectors

class StateSpaceSISO
{
public:
    // Default constructor, all matrices initialized to zero size
    StateSpaceSISO()
    {
        resize(0);
    }

    // Method to resize the system
    // Note: erases all parameters
    void resize(int size)
    {
        A.setZero(size, size);
        x.setZero(size);
        b.setZero(size);
        c.setZero(size);

        A_dim = 0;
        y = 0;
        u = 0;
    }

    void SetFromTransFcn(TransferFunction TF)
    {
        int TForder = TF.GetOrder();
        if (TForder > 0)
        {
            // Change to appropriate size
            resize(TForder);

            // Set A and c matrix coefficients
            for (int i = 0; i < TForder; i++)
            {
                if (i + 1 < TForder)
                    A(i, i + 1) = 1;
                A(TForder - 1, i) = -TF.dens(i);
                c(i) = TF.nums(i);
            }

            // Set b coefficients
            b(TForder - 1) = 1;
        }
    }

    // Creates matrices of specified sizes, and initializes values to zero
    StateSpaceSISO(int order) : A_dim(order)
    {
        A.setZero(A_dim, A_dim);
        x.setZero(A_dim);
        b.setZero(A_dim);
        c.setZero(A_dim);

        u = 0;
        y = 0;
    }

    // If dimensions of argument and A are equal, set matrix A
    // Error code of -1 is returned if dimensions are incorrect
    bool setA(Eigen::MatrixXd M)
    {
        if (M.rows() != A.rows())
        {
            return false;
        }
        if (M.cols() != A.cols())
        {
            return false;
        }
        A = M;
        return true;
    }

    // If dimensions of argument and b are equal, set matrix b
    // Error code of -1 is returned if dimensions are incorrect
    bool setB(Eigen::VectorXd M)
    {
        if (M.size() != b.size())
        {
            return false;
        }
        b = M;
        return true;
    }

    // If dimensions of argument and c are equal, set matrix c
    // Error code of -1 is returned if dimensions are incorrect
    bool setC(Eigen::VectorXd M)
    {
        if (M.size() != c.size())
        {
            return false;
        }
        c = M;
        return true;
    }

    // Method for propagating the differential equation through time
    void Propagate(double new_u, double dt)
    {
        u = new_u;
        x = x + (A * x + b * u) * dt;
    }

    // Computes and returns output y
    double getOutput()
    {
        y = c.dot(x);
        return y;
    }

    void HelpDebug()
    {
        cout << "Matrix A: " << endl << A << endl;
        cout << "Matrix B: " << endl << b << endl;
        cout << "Matrix C: " << endl << c << endl;
        cout << "Vector x: " << endl << x << endl;
        cout << "Vector y: " << endl << y << endl;
    }

private:
    int A_dim;

    Eigen::MatrixXd A;
    Eigen::VectorXd x;
    Eigen::VectorXd b;
    Eigen::VectorXd c;

    double u;
    double y;

};

#endif //ADAPTIVECTRL_STATE_SPACE_H
