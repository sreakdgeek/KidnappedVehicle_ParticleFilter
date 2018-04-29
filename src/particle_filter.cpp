/*
 *
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 

	num_particles = 101;

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	default_random_engine gen;

	for (int i = 0; i < num_particles; i++) {
		Particle particle;
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1;
		particles.push_back(particle);
		weights.push_back(particle.weight);
	}

	is_initialized = true;

	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	double new_x;
	double new_y;
	double new_theta;
	default_random_engine gen;


	for (int i = 0; i < num_particles; i++) {
		if (fabs(yaw_rate) < 0.0001)
		{
			new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
			new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
			new_theta = particles[i].theta;
		}
		else
		{
			new_x = particles[i].x + (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			new_y = particles[i].y + (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			new_theta = particles[i].theta + yaw_rate * delta_t;
		}

		normal_distribution<double> dist_x(new_x, std_pos[0]);
       		normal_distribution<double> dist_y(new_y, std_pos[1]);
       		normal_distribution<double> dist_theta(new_theta, std_pos[2]);
	

		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	//
	
	for (int i = 0; i < observations.size(); i++)
	{
		LandmarkObs obs = observations[i];
		double min_distance = std::numeric_limits<double>::max();
		int map_id = -1;

		for (int j = 0; j < predicted.size(); j++)
		{
			LandmarkObs pred = predicted[j];
			double l2_distance = dist(obs.x, obs.y, pred.x, pred.y);
			if (l2_distance < min_distance)
			{
				min_distance = l2_distance;
				map_id = pred.id;
			}

		}
		//std::cout << "Association for observations[" << i << "] = " << map_id << std::endl;
		observations[i].id = map_id;
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	//   Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	//
	// Create a vector of predicted measurements

	// Step # 1 - Create List of predictions
	
	double weight_normalizer = 0.0;
	
	for (int i = 0; i < num_particles; i++)
	{
		double p_x = particles[i].x;
		double p_y = particles[i].y;
		double p_theta = particles[i].theta;
		
		std::vector<LandmarkObs> predicted;

		for (int j = 0; j < map_landmarks.landmark_list.size(); j++)
		{
			int map_id = map_landmarks.landmark_list[j].id_i;
			float map_x = map_landmarks.landmark_list[j].x_f;
			float map_y = map_landmarks.landmark_list[j].y_f;

			/*double l2_distance = dist(p_x, p_y, map_x, map_y);

			if (l2_distance < sensor_range)
				predicted.push_back(LandmarkObs({map_id, map_x, map_y}));*/

			if ((fabs(map_x - p_x) <= sensor_range) && (fabs(map_y - p_y) <= sensor_range))
				predicted.push_back(LandmarkObs({map_id, map_x, map_y}));

		}

		//std::cout << "DEBUG: Number of predicted landmarks in sensor range for particle " << i << " = " << predicted.size() << std::endl;

		std::vector<LandmarkObs> transformed_observations;

		double obs_x;
		double obs_y;
		double pred_x;
		double pred_y;

		// Step # 2 - Create a list of transformed observations
		for (int k = 0; k < observations.size(); k++)
		{
			obs_x = p_x + (cos(p_theta) * observations[k].x  - sin(p_theta) * observations[k].y);
			obs_y = p_y + (sin(p_theta) * observations[k].x + cos(p_theta) * observations[k].y);
			transformed_observations.push_back(LandmarkObs({observations[k].id, obs_x, obs_y}));
			
		}

		// Step # 3 - Associate nearest prediction to transformed observation
		dataAssociation(predicted, transformed_observations);

		particles[i].weight = 1;
		bool match_found = false;

		// Step # 4 - Compute the weight of partilce using gaussian distance and update the weight of the particle
		for (int m = 0; m < transformed_observations.size(); m++)
		{
			obs_x = transformed_observations[m].x;
			obs_y = transformed_observations[m].y;

			for (int n = 0; n < predicted.size(); n++)
			{
				if (transformed_observations[m].id == predicted[n].id) {
					pred_x = predicted[n].x;
					pred_y = predicted[n].y;
					match_found = true;
					break;
				}
			}

			if (match_found) {

				double std_x = std_landmark[0];
				double std_y = std_landmark[1];
 
				double exponent = (((obs_x - pred_x) * (obs_x - pred_x)) / (2.0 * std_x * std_x)) + (((obs_y - pred_y) * (obs_y - pred_y)) / (2 * std_y * std_y));
				//std::cout << "DEBUG: Exponent computed for particle i = " << i << " = " << exponent << std::endl;

				particles[i].weight *= (1.0 / sqrt(2 * M_PI * std_x * std_y)) * exp(-1 * exponent);
			}
		}

		/*if (!match_found)
			particles[i].weight = 0;*/

		weight_normalizer += particles[i].weight;
	}

	// Normalizing weights in probability range of [0-1]
	for (int i = 0; i < num_particles; i++)
	{
		if (weight_normalizer != 0)
			particles[i].weight /= weight_normalizer;
		weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	//
	
	std::vector<double> weights;

	// Step # 1 - Fetch all the particle weights
	for (int i = 0; i < num_particles; i++)
		weights.push_back(particles[i].weight);

	/*double sum = std::accumulate(weights.begin(), weights.end(), 0);

	std::vector<double> new_weights;

	for (int i = 0; i < num_particles; i++) {
		if (sum == 0) 
			new_weights.push_back(weights[i]);
		else
			new_weights.push_back((double)weights[i]/sum);
	}

	weights = new_weights;*/

	// Step # 2 - Create uniform int distribution for the wheel index and uniform real distribution for weight
	std::default_random_engine gen;
  	std::uniform_int_distribution<int> dist_int(0, num_particles - 1);

	double max_weight = *std::max_element(weights.begin(), weights.end());

	//cout << "Max weight in re-sampling = " << max_weight << std::endl;


	double beta = 0.0;

	int index = dist_int(gen);

	std::vector<Particle> new_particles;

	// Step # 3 - Spin the wheel and find the particle that survives. Probabily of particles survival is proportional to its weight.
	for (int j = 0; j < num_particles; j++)
	{
		std::uniform_real_distribution<double> dist_real(0.0, max_weight);
		beta += dist_real(gen) * 2.0;

		while (beta > weights[index])
		{
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}

		// Surviving particle is particles[index]
		//std::cout << "Pushing surviving particle index = " << index << std::endl;
		new_particles.push_back(particles[index]);
	}

	// Step # 4 - Reset the new set of particles, which passed survival of the fittest (with some luck or god almighty if you are a believer) ;-)
	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates 
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
