/* Author: Anish Senathi
 *
 * - Implements the Bayes Filter for localization
 * - Updates the elevation and variance of the cell
 * - Returns the elevation and variance of the cell
 * */


#include "lx_mapping/bayes_filter.hpp"

BayesFilter::BayesFilter(){
    this->cellElevation = 0.0;
    this->cellVariance = 100.0;
}


void BayesFilter::updateCell(int z, float sigma_t){
    float z_hat = this->cellElevation;
    float sigma_hat = this->cellVariance;
    float z_t = z;  
    float k_t = sigma_hat/(sigma_hat + sigma_t);
    if(this->cellElevation == 0.0){
        k_t = 1.0;
    }
    this->cellElevation = z_hat + k_t*(z_t - z_hat);
    this->cellVariance = (1 - k_t)*sigma_hat;
    if(this->cellVariance < this->LOCALIZATION_VARIANCE){
        this->cellVariance = this->LOCALIZATION_VARIANCE;
    }
}

float BayesFilter::getCellElevation(){
    return this->cellElevation;
}

float BayesFilter::getCellVariance(){
    return this->cellVariance;
}
