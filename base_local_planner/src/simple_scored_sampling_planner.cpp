/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/simple_scored_sampling_planner.h>

#include <ros/console.h>

namespace base_local_planner {
  
  SimpleScoredSamplingPlanner::SimpleScoredSamplingPlanner(std::vector<TrajectorySampleGenerator*> gen_list, std::vector<TrajectoryCostFunction*>& critics, int max_samples) {
    max_samples_ = max_samples;
    gen_list_ = gen_list;
    critics_ = critics;

    ros::NodeHandle private_nh("~");
    private_nh.param("scored_sampling_threads_num", threads_num_, 1);
    //printf("SimpleScoredSamplingPlanner init\n");

    // spilt task into threads_num_ pieces and initial thread argument
    for (int i = 0; i < MAX_THREAD_TASK; i++) {
      for (int j = 0; j < threads_num_; j++) {
        range[j].case_index[i] = threads_num_ * i + j;
      }
    }
    for (int i = 0; i < threads_num_; i++)
      range[i].thread_index = i;
  }

  double SimpleScoredSamplingPlanner::scoreTrajectory(Trajectory& traj, double best_traj_cost) {
    double traj_cost = 0;
    int gen_id = 0;
    for(std::vector<TrajectoryCostFunction*>::iterator score_function = critics_.begin(); score_function != critics_.end(); ++score_function) {
      TrajectoryCostFunction* score_function_p = *score_function;
      if (score_function_p->getScale() == 0) {
        continue;
      }
      double cost = score_function_p->scoreTrajectory(traj);
      if (cost < 0) {
        ROS_DEBUG("Velocity %.3lf, %.3lf, %.3lf discarded by cost function  %d with cost: %f", traj.xv_, traj.yv_, traj.thetav_, gen_id, cost);
        traj_cost = cost;
        break;
      }
      if (cost != 0) {
        cost *= score_function_p->getScale();
      }
      traj_cost += cost;
      if (best_traj_cost > 0) {
        // since we keep adding positives, once we are worse than the best, we will stay worse
        if (traj_cost > best_traj_cost) {
          break;
        }
      }
      gen_id ++;
    }
    return traj_cost;
  }

  void* SimpleScoredSamplingPlanner::parallelTrajectory(void* _range) {

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */
    int i;
    bool gen_success;
    double loop_traj_cost;
    Trajectory loop_traj;

    ThreadsArg *range;
    range = (ThreadsArg *) _range;

    // real task for each thread
    for (i = 0; i < range->case_num; i++) {
      gen_success = range->gen_->nextTrajectory(loop_traj, range->case_index[i]);
      if (gen_success == false) {
        // TODO use this for debugging
        continue;
      }

      loop_traj_cost = range->this_planner->scoreTrajectory(loop_traj, range->best_cost);
      //printf("Traj %d cost: %f\n", i, loop_traj_cost);
      //if (range->all_explored != NULL) {
      //  loop_traj.cost_ = loop_traj_cost;
      //  range->all_explored->push_back(loop_traj);
      //}

      if (loop_traj_cost >= 0) {
        if (range->best_cost < 0 || loop_traj_cost < range->best_cost) {
          range->best_cost = loop_traj_cost;
          range->best_traj = loop_traj;
        }
      }
    }

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;

    printf("thread %d time: %lf\n", range->thread_index, t_diff);
    */

    pthread_exit(range);
  }

  bool SimpleScoredSamplingPlanner::findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored) {
    Trajectory best_traj;
    double best_traj_cost = -1;
    int count, count_valid, traj_size, i;
    void *ret;
    ThreadsArg* tmp_ret;
    //ret = new ThreadsArg;

    /* For timing uncomment
    static int count1 = 0;
    static double acc_time = 0;
    */

    for (std::vector<TrajectoryCostFunction*>::iterator loop_critic = critics_.begin(); loop_critic != critics_.end(); ++loop_critic) {
      TrajectoryCostFunction* loop_critic_p = *loop_critic;
      if (loop_critic_p->prepare() == false) {
        ROS_WARN("A scoring function failed to prepare");
        return false;
      }
    }

    for (std::vector<TrajectorySampleGenerator*>::iterator loop_gen = gen_list_.begin(); loop_gen != gen_list_.end(); ++loop_gen) {
      //count = 0;
      //count_valid = 0;
      TrajectorySampleGenerator* gen_ = *loop_gen;//TODO
      traj_size = gen_->getTrajectorySize();
      //ROS_DEBUG("Evaluated %d trajectories", traj_size);

      /* For timing uncomment
      struct timeval start, end;
      double start_t, end_t, t_diff;
      gettimeofday(&start, NULL);
      */

      // spilt task into threads_num_ pieces and initial thread argument
      int rem_temp = traj_size % threads_num_;
      for (i = 0; i < threads_num_; i++) {

        // record cases number per threads
        if (i < rem_temp)
          range[i].case_num = traj_size / threads_num_ + 1;
        else
          range[i].case_num = traj_size / threads_num_;

        range[i].best_cost = -1;
        range[i].gen_ = gen_->clone();//TODO
        //range[i].all_explored = all_explored;//TODO
        range[i].this_planner = this;//TODO
      }

      /* For timing uncomment
      gettimeofday(&end, NULL);
      start_t = start.tv_sec + double(start.tv_usec) / 1e6;
      end_t = end.tv_sec + double(end.tv_usec) / 1e6;
      t_diff = end_t - start_t;

      printf("spilt time: %lf\n", t_diff);
      */

      // for (i = 0; i < threads_num_; i++) {
      //   printf("-----------thread %d-----------\n", i);
      //   printf("case_num = %d\n", range[i].case_num);
      //   for (int j = 0; j < range[i].case_num; j++) {
      //     printf("%d ", range[i].case_index[j]);
      //     if (j % 10 == 0)
      //       printf("\n");
      //   }
      //   printf("\n");
      // }

      // threads create
      for (i = 0; i < threads_num_; i++) {
        if (pthread_create(threads + i, NULL, parallelTrajectory, &range[i])) {
          ROS_FATAL("Thread %d created fail.\n", i);
          exit(1);
        }
      }

      /* For timing uncomment
      gettimeofday(&start, NULL);
      */
      // waiting all threads finish tasks
      for(i = 0; i < threads_num_; i++) {
        int err = pthread_join(threads[i], &ret);
        if (err)
          perror("pthread_join failed: ");

        tmp_ret = (ThreadsArg *)ret;

        if (tmp_ret->best_cost >= 0) {
          if (best_traj_cost < 0 || tmp_ret->best_cost < best_traj_cost) {
            best_traj_cost = tmp_ret->best_cost;
            best_traj = tmp_ret->best_traj;
          }
        }
      }

      /* For timing uncomment
      gettimeofday(&end, NULL);
      start_t = start.tv_sec + double(start.tv_usec) / 1e6;
      end_t = end.tv_sec + double(end.tv_usec) / 1e6;
      t_diff = end_t - start_t;

      acc_time += t_diff;
      count1++;

      printf("thread_join time: %lf\n", t_diff);
      printf("---ACC thread_join time: %lf\n", acc_time/count1);
      */
      //printf("!!! best cost: %f\n\n\n", best_traj_cost);

      if (best_traj_cost >= 0) {
        traj.xv_ = best_traj.xv_;
        traj.yv_ = best_traj.yv_;
        traj.thetav_ = best_traj.thetav_;
        traj.cost_ = best_traj_cost;
        traj.resetPoints();
        double px, py, pth;
        for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
          best_traj.getPoint(i, px, py, pth);
          traj.addPoint(px, py, pth);
        }
      }
      //ROS_DEBUG("Evaluated %d trajectories, found %d valid", count, count_valid);
      if (best_traj_cost >= 0) {
        // do not try fallback generators
        break;
      }
    }
    return best_traj_cost >= 0;
  }
}
