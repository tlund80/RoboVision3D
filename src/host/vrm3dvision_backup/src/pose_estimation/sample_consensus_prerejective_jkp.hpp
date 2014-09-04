/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_REGISTRATION_SAMPLE_CONSENSUS_PREREJECTIVE_JKP_HPP_
#define PCL_REGISTRATION_SAMPLE_CONSENSUS_PREREJECTIVE_JKP_HPP_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
pcl::SampleConsensusPrerejectiveJKP<PointSource, PointTarget, FeatureT>::setSourceFeatures (const FeatureCloudConstPtr &features)
{
  if (features == NULL || features->empty ())
  {
    PCL_ERROR ("[pcl::%s::setSourceFeatures] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
    return;
  }
  input_features_ = features;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
pcl::SampleConsensusPrerejectiveJKP<PointSource, PointTarget, FeatureT>::setTargetFeatures (const FeatureCloudConstPtr &features)
{
  if (features == NULL || features->empty ())
  {
    PCL_ERROR ("[pcl::%s::setTargetFeatures] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
    return;
  }
  target_features_ = features;
  feature_tree_->setInputCloud (target_features_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//template <typename PointSource, typename PointTarget, typename FeatureT> void
//pcl::SampleConsensusPrerejectiveJKP<PointSource, PointTarget, FeatureT>::selectSamples (
//    const PointCloudSource &cloud, int nr_samples,
//    std::vector<int> &sample_indices)
//{
//  if (nr_samples > static_cast<int> (cloud.points.size ()))
//  {
//    PCL_ERROR ("[pcl::%s::selectSamples] ", getClassName ().c_str ());
//    PCL_ERROR ("The number of samples (%d) must not be greater than the number of points (%lu)!\n",
//               nr_samples, cloud.points.size ());
//    return;
//  }
//
//  // Iteratively draw random samples until nr_samples is reached
//  sample_indices.clear ();
//  while (static_cast<int> (sample_indices.size ()) < nr_samples)
//  {
//    // Choose a unique sample at random
//    int sample_index;
//
//    sample_index = getRandomIndex (static_cast<int> (cloud.points.size ()));
//
//    bool good_sample = true;
//    for (size_t i = 0; i < sample_indices.size (); i++)
//    {
//    	if (sample_indices[i] == sample_index)
//    	{
//    		good_sample = false;
//    		break;
//    	}
//    }
//
//    if (good_sample)
//    {
//    	// Store
//        sample_indices.push_back (sample_index);
//    }
//  }
//}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
pcl::SampleConsensusPrerejectiveJKP<PointSource, PointTarget, FeatureT>::selectSamples (
    const PointCloudSource &cloud, int nr_samples, 
    std::vector<int> &sample_indices, std::vector<int> &corresponding_indices, const std::vector<point_pair>& point_sets)
{
	{
	  if (nr_samples > static_cast<int> (cloud.points.size ()))
	  {
	    PCL_ERROR ("[pcl::%s::selectSamples] ", getClassName ().c_str ());
	    PCL_ERROR ("The number of samples (%d) must not be greater than the number of points (%lu)!\n",
	               nr_samples, cloud.points.size ());
	    return;
	  }

	  float min_sample_distance = 0.005;

	  // Iteratively draw random samples until nr_samples is reached
	  int iterations_without_a_sample = 0;
	  int max_iterations_without_a_sample = static_cast<int> (3 * cloud.points.size ());
	  sample_indices.clear ();
	  corresponding_indices.clear();
	  while (static_cast<int> (sample_indices.size ()) < nr_samples)
	  {
	    // Choose a sample at random
		float rand_val = (float)std::rand()/(float)RAND_MAX;

		int sample_index = 0;
		for (; sample_index < point_sets.size(); sample_index++)
		{
			if (rand_val <= point_sets[sample_index].accumulated_similarity)
				break;
		}

	    // Check to see if the sample is 1) unique and 2) far away from the other samples
	    bool valid_sample = true;
	    for (size_t i = 0; i < sample_indices.size (); ++i)
	    {
	      float distance_between_samples = pcl::euclideanDistance (cloud.points[point_sets[sample_index].first_idx], cloud.points[point_sets[sample_index].second_idx]);

	      if (point_sets[sample_index].first_idx == sample_indices[i] || point_sets[sample_index].second_idx == corresponding_indices[i] || distance_between_samples < min_sample_distance)
	      {
	        valid_sample = false;
	        break;
	      }
	    }

	    // If the sample is valid, add it to the output
	    if (valid_sample)
	    {
	      sample_indices.push_back (point_sets[sample_index].first_idx);
	      corresponding_indices.push_back(point_sets[sample_index].second_idx);
	      iterations_without_a_sample = 0;
	    }
	    else
	      ++iterations_without_a_sample;

	    // If no valid samples can be found, relax the inter-sample distance requirements
	    if (iterations_without_a_sample >= max_iterations_without_a_sample)
	    {
	      PCL_WARN ("[pcl::%s::selectSamples] ", getClassName ().c_str ());
	      PCL_WARN ("No valid sample found after %d iterations. Relaxing min_sample_distance_ to %f\n",
		                iterations_without_a_sample, 0.5*min_sample_distance);

	      min_sample_distance *= 0.5f;
	      iterations_without_a_sample = 0;
	    }
	  }
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
pcl::SampleConsensusPrerejectiveJKP<PointSource, PointTarget, FeatureT>::findSimilarFeatures (
    const FeatureCloud &input_features, const std::vector<int> &sample_indices, std::vector<std::vector<int> >& similar_features,
    std::vector<int> &corresponding_indices)
{
	std::vector<float> nn_distances (k_correspondences_);

	corresponding_indices.resize (sample_indices.size ());
	for (size_t i = 0; i < sample_indices.size (); ++i)
	{
		if (similar_features[sample_indices[i]].empty())
		{
			// Find the k features nearest to input_features.points[sample_indices[i]]
			feature_tree_->nearestKSearch (input_features, sample_indices[i], k_correspondences_, similar_features[sample_indices[i]], nn_distances);
		}

		// Select one at random and add it to corresponding_indices
		if (k_correspondences_ == 1)
		{
		  corresponding_indices[i] = similar_features[sample_indices[i]][0];
		}
		else
		{
			int random_correspondence = getRandomIndex (k_correspondences_);
			corresponding_indices[i] = similar_features[sample_indices[i]][random_correspondence];
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
pcl::SampleConsensusPrerejectiveJKP<PointSource, PointTarget, FeatureT>::computeTransformation (PointCloudSource &output, const Eigen::Matrix4f& guess)
{
  // Some sanity checks first
  if (!input_features_)
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
    PCL_ERROR ("No source features were given! Call setSourceFeatures before aligning.\n");
    return;
  }
  if (!target_features_)
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
    PCL_ERROR ("No target features were given! Call setTargetFeatures before aligning.\n");
    return;
  }

  if (input_->size () != input_features_->size ())
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
    PCL_ERROR ("The source points and source feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
               input_->size (), input_features_->size ());
    return;
  }

  if (target_->size () != target_features_->size ())
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
    PCL_ERROR ("The target points and target feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
               target_->size (), target_features_->size ());
    return;
  }

  if (inlier_fraction_ < 0.0f || inlier_fraction_ > 1.0f)
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
    PCL_ERROR ("Illegal inlier fraction %f, must be in [0,1]!\n",
               inlier_fraction_);
    return;
  }
  
  const float similarity_threshold = correspondence_rejector_poly_->getSimilarityThreshold ();
  if (similarity_threshold < 0.0f || similarity_threshold >= 1.0f)
  {
    PCL_ERROR ("[pcl::%s::computeTransformation] ", getClassName ().c_str ());
    PCL_ERROR ("Illegal prerejection similarity threshold %f, must be in [0,1[!\n",
               similarity_threshold);
    return;
  }
  
  // Initialize prerejector (similarity threshold already set to default value in constructor)
  correspondence_rejector_poly_->setInputSource (input_);
  correspondence_rejector_poly_->setInputTarget (target_);
  correspondence_rejector_poly_->setCardinality (nr_samples_);
  std::vector<bool> accepted (input_->size (), false); // Indices of sampled points that passed prerejection
  int num_rejections = 0; // For debugging
  
  // Initialize results
  final_transformation_ = guess;
  inliers_.clear ();
  float lowest_error = std::numeric_limits<float>::max ();
  converged_ = false;

  // Temporaries
  std::vector<int> inliers;
  float inlier_fraction;
  float error;
  
  int it = 0;
  int conv = 0;
  int fitnesses = 0;

  double select_sample_time = 0;
  double find_similar_time = 0;
  double prereject_time = 0;
  double fitness_time = 0;

  struct timeval start;
  struct timeval total;
  struct timeval now;

  gettimeofday(&total, NULL);

  std::vector<std::vector<int> > similar_features(input_->size ());


  gettimeofday(&start, NULL);

  std::vector<point_pair> point_sets(input_->size() * k_correspondences_);
  float total_similarity = 0;
  for (int i = 0; i < input_->size() ;i++)
  {
	  std::vector<int> nearby_features (k_correspondences_);
	  std::vector<float> nearby_distances (k_correspondences_);
	  feature_tree_->nearestKSearch (*input_features_, i, k_correspondences_, nearby_features, nearby_distances);

	  for (int j = 0; j < k_correspondences_; j++)
	  {
		  total_similarity += 1.0 / nearby_distances[j];

		  point_sets[i*k_correspondences_ + j].first_idx = i;
		  point_sets[i*k_correspondences_ + j].second_idx = nearby_features[j];
		  point_sets[i*k_correspondences_ + j].similarity = 1.0 / nearby_distances[j];
		  point_sets[i*k_correspondences_ + j].accumulated_similarity = total_similarity;
	  }
  }

  for (int i = 0; i < point_sets.size(); i++)
  {
	  point_sets[i].similarity /= total_similarity;
	  point_sets[i].accumulated_similarity /= total_similarity;

	  //std::cout << "I1: " << point_sets[i].first_idx << " I2: " << point_sets[i].second_idx << " Sim: " << point_sets[i].similarity << " AccSim: " << point_sets[i].accumulated_similarity << std::endl;
  }

  gettimeofday(&now, NULL);
  find_similar_time = double((now.tv_sec * 1000000 + now.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec)) / 1000.0;

  // Start
  for (int i = 0; i < max_iterations_; ++i)
  {
	it++;
    // Temporary containers
    std::vector<int> sample_indices (nr_samples_);
    std::vector<int> corresponding_indices (nr_samples_);

    gettimeofday(&start, NULL);
    // Draw nr_samples_ random samples
    selectSamples (*input_, nr_samples_, sample_indices, corresponding_indices, point_sets);
    gettimeofday(&now, NULL);
    select_sample_time += double(now.tv_usec - start.tv_usec) / 1000.0;
    
//    // Check if all sampled points already been accepted
//    bool samples_accepted = true;
//    for (unsigned int j = 0; j < sample_indices.size(); ++j) {
//      if (!accepted[sample_indices[j]]) {
//        samples_accepted = false;
//        break;
//      }
//    }
//
//    // All points have already been accepted, avoid
//    if (samples_accepted)
//      continue;

    // Apply prerejection
    bool not_rejected = correspondence_rejector_poly_->thresholdPolygon (sample_indices, corresponding_indices);

    if (!not_rejected) {
      ++num_rejections;
      continue;
    }

    // Estimate the transform from the correspondences, write to transformation_
    transformation_estimation_->estimateRigidTransformation (*input_, sample_indices, *target_, corresponding_indices, transformation_);
    
    // Take a backup of previous result
    const Matrix4 final_transformation_prev = final_transformation_;
    
    // Set final result to current transformation
    final_transformation_ = transformation_;
    
    gettimeofday(&start, NULL);
    // Transform the input and compute the error (uses input_ and final_transformation_)
    getFitness (inliers, error);
    fitnesses++;
    gettimeofday(&now, NULL);
    fitness_time += double((now.tv_sec * 1000000 + now.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec)) / 1000.0;

    // Restore previous result
    final_transformation_ = final_transformation_prev;

    // If the new fit is better, update results
    inlier_fraction = static_cast<float> (inliers.size ()) / static_cast<float> (input_->size ());
    
    if (inlier_fraction >= inlier_fraction_) {
      // Mark the sampled points accepted
      for (int j = 0; j < nr_samples_; ++j)
        accepted[sample_indices[j]] = true;
      
      // Update result if pose hypothesis is better
      if (error < lowest_error) {
        inliers_ = inliers;
        conv++;
        lowest_error = error;
        converged_ = true;
        final_transformation_ = transformation_;
      }

      // If inlier fraction is high enough then stop.
      if (inlier_fraction > inlier_fraction_termination_)
      {
    	  break;
      }
    }
  }

  gettimeofday(&now, NULL);
//  std::cout << "*** SCP converged: " << conv << " in iterations: " << it << " - Prerejected: " << num_rejections << " Fitness calculated: " << fitnesses <<  std::endl;
//  std::cout << "*** total time: " << double((now.tv_sec * 1000000 + now.tv_usec) - (total.tv_sec * 1000000 + total.tv_usec)) / 1000.0 << " ms - Select_sample: " << select_sample_time << " ms - find similar features: " << find_similar_time << " ms - prereject: " << prereject_time << " ms - Fitness: " << fitness_time << " ms" << std::endl;

  // Apply the final transformation
  if (converged_)
    transformPointCloud (*input_, output, final_transformation_);
  
  // Debug output
  PCL_DEBUG("[pcl::%s::computeTransformation] Rejected %i out of %i generated pose hypotheses.\n",
            getClassName ().c_str (), num_rejections, max_iterations_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> void 
pcl::SampleConsensusPrerejectiveJKP<PointSource, PointTarget, FeatureT>::getFitness (std::vector<int>& inliers, float& fitness_score)
{
  // Initialize variables
  inliers.clear ();
  inliers.reserve (input_->size ());
  fitness_score = 0.0f;
  
  // Use squared distance for comparison with NN search results
  const float max_range = corr_dist_threshold_ * corr_dist_threshold_;

  // Transform the input dataset using the final transformation
  PointCloudSource input_transformed;
  input_transformed.resize (input_->size ());
  transformPointCloud (*input_, input_transformed, final_transformation_);
  
  // For each point in the source dataset
  for (size_t i = 0; i < input_transformed.points.size (); ++i)
  {
    // Find its nearest neighbor in the target
    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);
    tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);
    
    // Check if point is an inlier
    if (nn_dists[0] < max_range)
    {
      // Update inliers
      inliers.push_back (static_cast<int> (i));
      fitness_score += nn_dists[0];
    }
  }

  // Calculate MSE
  if (inliers.size () > 0)
    fitness_score /= static_cast<float> (inliers.size ());
  else
    fitness_score = std::numeric_limits<float>::max ();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename FeatureT> int
pcl::SampleConsensusPrerejectiveJKP<PointSource, PointTarget, FeatureT>::recalculateInliers (const Eigen::Matrix4f& guess)
{
	const float max_range = corr_dist_threshold_ * corr_dist_threshold_;
	int inlier_count = 0;
	// Transform the input dataset using the final transformation
	PointCloudSource input_transformed;
	input_transformed.resize (input_->size ());
	transformPointCloud (*input_, input_transformed, guess);

	// For each point in the source dataset
	for (size_t i = 0; i < input_transformed.points.size (); ++i)
	{
		// Find its nearest neighbor in the target
		std::vector<int> nn_indices (1);
		std::vector<float> nn_dists (1);
		tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);

		// Check if point is an inlier
		if (nn_dists[0] < max_range)
		{
			inlier_count++;
		}
	}
	return inlier_count;
}

#endif

