#ifndef PHYSICS_H
#define PHYSICS_H

#include <ros/ros.h>
#include <tuple>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <math.h>

enum Field
{
    X_VELOCITY_FIELD,
    Y_VELOCITY_FIELD,
    Z_VELOCITY_FIELD,
    X_VEL_FIELD,
    Y_VEL_FIELD,
    Z_VEL_FIELD,
    S_FIELD
};

class Fluid
{
public:
    int density;
    int numX;
    int numY;
    int numZ;
    int numCells;
    double h;
    double overRelaxation;
    double *x_velocity;
    double *y_velocity;
    double *z_velocity;
    double *new_x_velocity;
    double *new_y_velocity;
    double *new_z_velocity;
    int *scaler_value;
    double *density_mesh;
    double *new_density_mesh;
    double *pressure;

    Fluid() {}

    Fluid(int density, int numX, int numY, int numZ, double h)
    {
        this->density = density;
        this->overRelaxation = 1.9;
        this->numX = numX + 2;
        this->numY = numY + 2;
        this->numZ = numZ + 2;
        this->numCells = this->numX * this->numY * this->numZ;
        this->h = h;
        this->x_velocity = new double[this->numCells];
        this->y_velocity = new double[this->numCells];
        this->z_velocity = new double[this->numCells];
        this->new_x_velocity = new double[this->numCells];
        this->new_y_velocity = new double[this->numCells];
        this->new_z_velocity = new double[this->numCells];
        this->scaler_value = new int[this->numCells];
        this->density_mesh = new double[this->numCells];
        this->new_density_mesh = new double[this->numCells];
        std::fill(this->density_mesh, this->density_mesh + this->numCells, 1.0);
        std::fill(this->scaler_value, this->scaler_value + (int)(this->numCells * 0.9), 1);
    }

    void Integrate(double dt, double gravity)
    {
        auto columnNum = this->numY;
        auto xy_slice = this->numX * this->numY;
        int z,i,j;

#pragma omp parallel shared(columnNum, xy_slice, scaler_value, z_velocity, dt, gravity)
        {
#pragma omp for private(z, i, j)
            for (z = 1; z < this->numZ - 1; z++)
            {
                for (i = 1; i < this->numX - 1; i++)
                {
                    for (j = 1; j < this->numY - 1; j++)
                    {
                        if (this->scaler_value[z * xy_slice + i * columnNum + j] != 0.0 && this->scaler_value[(z - 1) * xy_slice + i * columnNum + j] != 0.0)
                            this->z_velocity[z * xy_slice + i * columnNum + j] += dt * gravity;
                    }
                }
            }
    }
}

void
SolveIncompressibility(int numIters, double dt)
{

    auto columnNum = this->numY;
    auto xy_slice = this->numX * this->numY;
    auto cp = this->density * this->h / dt;
    int z_i, i, j, iter;

#pragma omp parallel shared(scaler_value, x_velocity, y_velocity, z_velocity, columnNum, xy_slice   )
{
#pragma omp for private(z_i, i, j, iter)
    for (iter = 0; iter < numIters; iter++)
    {
        for (z_i = 1; z_i < this->numZ - 1; z_i++)
        {
            for (i = 1; i < this->numX - 1; i++)
            {
                for (j = 1; j < this->numY - 1; j++)
                {
                    int idx = z_i * xy_slice + i * columnNum + j;
                    if (this->scaler_value[idx] == 0.0)
                        continue;

                    int left_idx = idx - columnNum;
                    int right_idx = idx + columnNum;
                    int down_idx = idx - 1;
                    int up_idx = idx + 1;
                    int back_idx = idx - xy_slice;
                    int front_idx = idx + xy_slice;

                    auto sx0 = this->scaler_value[left_idx];
                    auto sx1 = this->scaler_value[right_idx];
                    auto sy0 = this->scaler_value[down_idx];
                    auto sy1 = this->scaler_value[up_idx];
                    auto sz0 = this->scaler_value[back_idx];
                    auto sz1 = this->scaler_value[front_idx];
                    auto s = sx0 + sx1 + sy0 + sy1 + sz0 + sz1;
                    if (s == 0.0)
                        continue;

                    auto div = this->x_velocity[right_idx] - this->x_velocity[idx] +
                               this->y_velocity[up_idx] - this->y_velocity[idx] +
                               this->z_velocity[front_idx] - this->z_velocity[idx];

                    auto p = -div / s;
                    p *= this->overRelaxation;

                    this->x_velocity[idx] -= sx0 * p;
                    this->x_velocity[right_idx] += sx1 * p;
                    this->y_velocity[idx] -= sy0 * p;
                    this->y_velocity[up_idx] += sy1 * p;
                    this->z_velocity[idx] -= sz0 * p;
                    this->z_velocity[front_idx] += sz1 * p;
                }
            }
        }
    }
}
}

double weightedAvg(double x, double y, double z, Field field)
{
    auto columnNum = this->numY;
    auto xy_slice = this->numX * this->numY;
    auto h = this->h;
    auto reversed_h = 1.0 / h;
    auto half_h = 0.5 * h;

    x = std::max(std::min(x, this->numX * h), h);
    y = std::max(std::min(y, this->numY * h), h);
    z = std::max(std::min(z, this->numZ * h), h);

    auto dx = 0.0;
    auto dy = 0.0;
    auto dz = 0.0;

    double *f;

    switch (field)
    {
    case X_VEL_FIELD:
        f = this->x_velocity;
        dy = half_h;
        dz = half_h;
        break;
    case Y_VEL_FIELD:
        f = this->y_velocity;
        dx = half_h;
        dz = half_h;
        break;
    case Z_VEL_FIELD:
        f = this->z_velocity;
        dx = half_h;
        dy = half_h;
        break;
    case S_FIELD:
        f = this->density_mesh;
        dx = half_h;
        dy = half_h;
        dz = half_h;
        break;
    }

    auto x0 = std::min((int)floor((x - dx) * reversed_h), this->numX - 1);
    auto weight_x1 = ((x - dx) - x0 * h) * reversed_h;
    auto x1 = std::min(x0 + 1, this->numX - 1);

    auto y0 = std::min((int)floor((y - dy) * reversed_h), this->numY - 1);
    auto weight_y1 = ((y - dy) - y0 * h) * reversed_h;
    auto y1 = std::min(y0 + 1, this->numY - 1);

    auto z0 = std::min((int)floor((z - dz) * reversed_h), this->numZ - 1);
    auto weight_z1 = ((z - dz) - z0 * h) * reversed_h;
    auto z1 = std::min(z0 + 1, this->numZ - 1);

    auto weight_x2 = 1.0 - weight_x1;
    auto weight_y2 = 1.0 - weight_y1;
    auto weight_z2 = 1.0 - weight_z1;

    auto weighted_avg =
        weight_z2 * weight_x2 * weight_y2 * f[z0 * xy_slice + x0 * columnNum + y0] +
        weight_z2 * weight_x1 * weight_y2 * f[z0 * xy_slice + x1 * columnNum + y0] +
        weight_z2 * weight_x1 * weight_y1 * f[z0 * xy_slice + x1 * columnNum + y1] +
        weight_z2 * weight_x2 * weight_y1 * f[z0 * xy_slice + x0 * columnNum + y1] +
        weight_z1 * weight_x2 * weight_y2 * f[z1 * xy_slice + x0 * columnNum + y0] +
        weight_z1 * weight_x1 * weight_y2 * f[z1 * xy_slice + x1 * columnNum + y0] +
        weight_z1 * weight_x1 * weight_y1 * f[z1 * xy_slice + x1 * columnNum + y1] +
        weight_z1 * weight_x2 * weight_y1 * f[z1 * xy_slice + x0 * columnNum + y1];

    return weighted_avg;
}

double avgX_forY(int z_i, int i, int j)
{
    auto columnNum = this->numY;
    auto xy_slice = this->numX * this->numY;

    int idx = z_i * xy_slice + i * columnNum + j;
    int down_idx = idx - 1;
    int right_down_idx = idx + columnNum - 1;
    int right_idx = idx + columnNum;

    auto x_vel = (this->x_velocity[down_idx] + this->x_velocity[idx] +
                  this->x_velocity[right_down_idx] + this->x_velocity[right_idx]) *
                 0.25;
    return x_vel;
}

double avgZ_forY(int z_i, int i, int j)
{
    auto columnNum = this->numY;
    auto xy_slice = this->numX * this->numY;

    int idx = z_i * xy_slice + i * columnNum + j;
    int back_idx = idx - xy_slice;
    int back_right_idx = idx - xy_slice + columnNum;
    int right_idx = idx + columnNum;

    auto z_vel = (this->z_velocity[back_idx] + this->z_velocity[idx] +
                  this->z_velocity[back_right_idx] + this->z_velocity[right_idx]) *
                 0.25;
    return z_vel;
}

double avgY_forX(int z_i, int i, int j)
{
    auto columnNum = this->numY;
    auto xy_slice = this->numX * this->numY;

    int idx = z_i * xy_slice + i * columnNum + j;
    int left_idx = idx - columnNum;
    int up_idx = idx + 1;
    int left_up_idx = idx - columnNum + 1;

    auto y_vel = (this->y_velocity[left_idx] + this->y_velocity[idx] +
                  this->y_velocity[left_up_idx] + this->y_velocity[up_idx]) *
                 0.25;
    return y_vel;
}

double avgZ_forX(int z_i, int i, int j)
{
    auto columnNum = this->numY;
    auto xy_slice = this->numX * this->numY;

    int idx = z_i * xy_slice + i * columnNum + j;
    int back_idx = idx - xy_slice;
    int up_idx = idx + 1;
    int back_up_idx = idx - xy_slice + 1;

    auto z_vel = (this->z_velocity[back_idx] + this->z_velocity[idx] +
                  this->z_velocity[back_up_idx] + this->z_velocity[up_idx]) *
                 0.25;
    return z_vel;
}

double avgX_forZ(int z_i, int i, int j)
{
    auto columnNum = this->numY;
    auto xy_slice = this->numX * this->numY;

    int idx = z_i * xy_slice + i * columnNum + j;
    int front_idx = idx + xy_slice;
    int down_idx = idx - 1;
    int front_down_idx = idx + xy_slice - 1;

    auto x_vel = (this->x_velocity[front_idx] + this->x_velocity[idx] +
                  this->x_velocity[front_down_idx] + this->x_velocity[down_idx]) *
                 0.25;
    return x_vel;
}

double avgY_forZ(int z_i, int i, int j)
{
    auto columnNum = this->numY;
    auto xy_slice = this->numX * this->numY;

    int idx = z_i * xy_slice + i * columnNum + j;
    int front_idx = idx + xy_slice;
    int left_idx = idx - columnNum;
    int front_left_idx = idx + xy_slice - columnNum;

    auto y_vel = (this->y_velocity[front_idx] + this->y_velocity[idx] +
                  this->y_velocity[front_left_idx] + this->y_velocity[left_idx]) *
                 0.25;
    return y_vel;
}

void advectVel(double dt)
{
    std::copy(this->x_velocity, this->x_velocity + this->numCells, this->new_x_velocity);
    std::copy(this->y_velocity, this->y_velocity + this->numCells, this->new_y_velocity);
    std::copy(this->z_velocity, this->z_velocity + this->numCells, this->new_z_velocity);

    auto columnNum = this->numY;
    auto xy_slice = this->numX * this->numY;
    auto h = this->h;
    auto half_h = 0.5 * h;
    int z_i, i, j;

#pragma omp parallel shared(scaler_value, x_velocity, y_velocity, z_velocity, columnNum, xy_slice, new_x_velocity, new_y_velocity, new_z_velocity, h, half_h)
{
#pragma omp for private(z_i, i, j)
    for (z_i = 1; z_i < this->numZ; z_i++)
    {
        for (i = 1; i < this->numX; i++)
        {
            for (j = 1; j < this->numY; j++)
            {
                int idx = z_i * xy_slice + i * columnNum + j;
                if (this->scaler_value[idx] == 0.0)
                    continue;

                int left_idx = idx - columnNum;
                // x velocity
                if (this->scaler_value[left_idx] != 0.0 && j < this->numY - 1 && z_i < this->numZ - 1)
                {
                    auto x = i * h;
                    auto y = j * h + half_h;
                    auto z = z_i * h + half_h;
                    auto x_vel = this->x_velocity[idx];
                    auto y_vel = this->avgY_forX(z_i, i, j);
                    auto z_vel = this->avgZ_forX(z_i, i, j);
                    x = x - dt * x_vel;
                    y = y - dt * y_vel;
                    z = z - dt * z_vel;
                    x_vel = this->weightedAvg(x, y, z, X_VEL_FIELD);
                    this->new_x_velocity[idx] = x_vel;
                }
                int down_idx = idx - 1;
                // y velocity
                if (this->scaler_value[down_idx] != 0.0 && i < this->numX - 1 && z_i < this->numZ - 1)
                {
                    auto x = i * h + half_h;
                    auto y = j * h;
                    auto z = z_i * h + half_h;
                    auto x_vel = this->avgX_forY(z_i, i, j);
                    auto y_vel = this->y_velocity[idx];
                    auto z_vel = this->avgZ_forY(z_i, i, j);
                    x = x - dt * x_vel;
                    y = y - dt * y_vel;
                    z = z - dt * z_vel;
                    y_vel = this->weightedAvg(x, y, z, Y_VEL_FIELD);
                    this->new_y_velocity[idx] = y_vel;
                }
                int back_idx = idx - xy_slice;
                // z velocity
                if (this->scaler_value[back_idx] != 0.0 && i < this->numX - 1 && j < this->numY - 1)
                {
                    auto x = i * h + half_h;
                    auto y = j * h + half_h;
                    auto z = z_i * h;
                    auto x_vel = this->avgX_forZ(z_i, i, j);
                    auto y_vel = this->avgY_forZ(z_i, i, j);
                    auto z_vel = this->z_velocity[idx];
                    x = x - dt * x_vel;
                    y = y - dt * y_vel;
                    z = z - dt * z_vel;
                    z_vel = this->weightedAvg(x, y, z, Z_VEL_FIELD);
                    this->new_z_velocity[idx] = z_vel;
                }
            }
        }
    }
}

    std::copy(this->new_x_velocity, this->new_x_velocity + this->numCells, this->x_velocity);
    std::copy(this->new_y_velocity, this->new_y_velocity + this->numCells, this->y_velocity);
    std::copy(this->new_z_velocity, this->new_z_velocity + this->numCells, this->z_velocity);
}

void advectSmoke(double dt)
{
    std::copy(this->density_mesh, this->density_mesh + this->numCells, this->new_density_mesh);

    auto columnNum = this->numY;
    auto xy_slice = this->numX * this->numY;
    auto h = this->h;
    auto half_h = 0.5 * h;
    int z_i, i, j;

#pragma omp parallel shared(scaler_value, x_velocity, y_velocity, z_velocity, new_density_mesh, columnNum, xy_slice, h, half_h)
{
#pragma omp for private(z_i, i, j)
    for (z_i = 1; z_i < this->numZ - 1; z_i++)
    {
        for (i = 1; i < this->numX - 1; i++)
        {
            for (j = 1; j < this->numY - 1; j++)
            {
                int idx = z_i * xy_slice + i * columnNum + j;
                if (this->scaler_value[idx] == 0.0)
                    continue;

                int right_idx = idx + columnNum;
                int up_idx = idx + 1;
                int front_idx = idx + xy_slice;

                auto x_vel = (this->x_velocity[idx] + this->x_velocity[right_idx]) * 0.5;
                auto y_vel = (this->y_velocity[idx] + this->y_velocity[up_idx]) * 0.5;
                auto z_vel = (this->z_velocity[idx] + this->z_velocity[front_idx]) * 0.5;

                auto x = i * h + half_h - dt * x_vel;
                auto y = j * h + half_h - dt * y_vel;
                auto z = z_i * h + half_h - dt * z_vel;

                this->new_density_mesh[idx] = this->weightedAvg(x, y, z, S_FIELD);
            }
        }
    }
}
    std::copy(this->new_density_mesh, this->new_density_mesh + this->numCells, this->density_mesh);
}

double getConcentration(double x, double y, double z)
{
    return this->weightedAvg(x, y, z, S_FIELD);
}

// ----------------- end of simulator ------------------------------

void simulate(double dt, double gravity, int numIters)
{
    this->Integrate(dt, gravity);
    this->SolveIncompressibility(numIters, dt);
    this->advectVel(dt);
    this->advectSmoke(dt);
}
}
;

#endif // PHYSICS_H
