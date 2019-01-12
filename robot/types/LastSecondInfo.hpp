#pragma once

#include <boost/serialization/version.hpp>

// Collection of vision info from the last second
struct LastSecondInfo {
   // Stores UNIX timestamp at the beginning frame
   int64_t initial_timestamp;

   // The features currently being collected
   int next_num_frames;

   int next_num_corners;
   int next_min_corners_distance;
   int next_avg_corners_distance;
   int next_max_corners_distance;


   int next_num_t_junctions;
   int next_min_t_junctions_distance;
   int next_avg_t_junctions_distance;
   int next_max_t_junctions_distance;

   int next_num_posts;
   int next_min_posts_kDistance;
   int next_avg_posts_kDistance;
   int next_max_posts_kDistance;

   int next_min_posts_wDistance;
   int next_avg_posts_wDistance;
   int next_max_posts_wDistance;

   int next_num_robots;

   int next_num_balls;
   int next_min_balls_distance;
   int next_avg_balls_distance;
   int next_max_balls_distance;

   int next_num_lines;

   int next_num_penalty_spots;
   int next_num_centre_circles;
   int next_num_field_line_points;

   // Total amount of features collected in the last second
   int num_frames;

   int num_corners;
   double num_corners_per_frame;
   int min_corners_distance;
   int avg_corners_distance;
   int max_corners_distance;

   int num_t_junctions;
   double num_t_junctions_per_frame;
   int min_t_junctions_distance;
   int avg_t_junctions_distance;
   int max_t_junctions_distance;

   int num_posts;
   double num_posts_per_frame;
   int min_posts_kDistance;
   int avg_posts_kDistance;
   int max_posts_kDistance;
   int min_posts_wDistance;
   int avg_posts_wDistance;
   int max_posts_wDistance;

   int num_robots;
   double num_robots_per_frame;

   int num_balls;
   double num_balls_per_frame;
   int min_balls_distance;
   int avg_balls_distance;
   int max_balls_distance;

   int num_lines;
   double num_lines_per_frame;

   int num_penalty_spots;
   double num_penalty_spots_per_frame;
   int num_centre_circles;
   double num_centre_circles_per_frame;
   int num_field_line_points;
   double num_field_line_points_per_frame;

   LastSecondInfo() {
      initial_timestamp = 0;

      // Features currently being collected
      next_num_frames = 0;

      next_num_corners = 0;
      next_min_corners_distance = 0;
      next_avg_corners_distance = 0;
      next_max_corners_distance = 0;
      
      next_num_t_junctions = 0;
      next_min_t_junctions_distance = 0;
      next_avg_t_junctions_distance = 0;
      next_max_t_junctions_distance = 0;

      next_num_posts = 0;
      next_min_posts_kDistance = 0;
      next_avg_posts_kDistance = 0;
      next_max_posts_kDistance = 0;

      next_min_posts_wDistance = 0;
      next_avg_posts_wDistance = 0;
      next_max_posts_wDistance = 0;

      next_num_robots = 0;

      next_num_balls = 0;
      next_min_balls_distance = 0;
      next_avg_balls_distance = 0;
      next_max_balls_distance = 0;      

      next_num_lines = 0;

      next_num_penalty_spots = 0;
      next_num_centre_circles = 0;
      next_num_field_line_points = 0;

      // Features collected in the last second
      num_frames = 0;

      num_corners = 0;
      num_corners_per_frame = 0;
      min_corners_distance = 0;
      avg_corners_distance = 0;
      max_corners_distance = 0;

      num_t_junctions = 0;
      num_t_junctions_per_frame = 0;
      min_t_junctions_distance = 0;
      avg_t_junctions_distance = 0;
      max_t_junctions_distance = 0;

      num_posts = 0;
      num_posts_per_frame = 0;
      num_posts_per_frame = 0;
      min_posts_kDistance = 0;
      avg_posts_kDistance = 0;
      max_posts_kDistance = 0;
      min_posts_wDistance = 0;
      avg_posts_wDistance = 0;
      max_posts_wDistance = 0;
      
      num_robots = 0;
      num_robots_per_frame = 0;

      num_balls = 0;
      num_balls_per_frame = 0;
      min_balls_distance = 0;
      avg_balls_distance = 0;
      max_balls_distance = 0;
      
      num_lines = 0;
      num_lines_per_frame = 0;

      num_penalty_spots = 0;
      num_penalty_spots_per_frame = 0;
      num_centre_circles = 0;
      num_centre_circles_per_frame = 0;
      num_field_line_points = 0;
      num_field_line_points_per_frame = 0;
   }
   
   void reset() {

      // Set the info taken from the last second
      num_frames = next_num_frames;

      num_corners_per_frame = (next_num_frames > 0) ?
                     (double) next_num_corners/next_num_frames : 0;
      num_corners = next_num_corners;
      min_corners_distance = next_min_corners_distance;
      avg_corners_distance = (next_num_corners > 0) ? 
                              next_avg_corners_distance/next_num_corners : 0;
      max_corners_distance = next_max_corners_distance;
      
      num_t_junctions_per_frame = (next_num_frames > 0) ? 
                         (double) next_num_t_junctions/next_num_frames : 0;
      num_t_junctions = next_num_t_junctions;
      min_t_junctions_distance = next_min_t_junctions_distance;
      avg_t_junctions_distance = (next_num_t_junctions > 0) ? 
                                  next_avg_t_junctions_distance/next_num_t_junctions : 0;
      max_t_junctions_distance = next_max_t_junctions_distance;

      num_posts = next_num_posts; 
      num_posts_per_frame = (next_num_frames > 0) ?
                   (double) next_num_posts/next_num_frames : 0;
      min_posts_kDistance = next_min_posts_kDistance;
      avg_posts_kDistance = (next_num_posts > 0) ?
                             next_avg_posts_kDistance/next_num_posts : 0;
      max_posts_kDistance = next_max_posts_kDistance;

      min_posts_wDistance = next_min_posts_wDistance;
      avg_posts_wDistance = (next_num_posts > 0) ?
                             next_avg_posts_wDistance/next_num_posts : 0;
      max_posts_wDistance = next_max_posts_wDistance;

      num_robots_per_frame = (next_num_frames > 0) ? 
                    (double) next_num_robots/next_num_frames : 0;
      num_robots = next_num_robots;

      num_balls_per_frame = (next_num_frames > 0) ? 
                   (double) next_num_balls/next_num_frames : 0;
      num_balls = next_num_balls;
      min_balls_distance = next_min_balls_distance;
      avg_balls_distance = (next_num_balls > 0) ?
                            next_avg_balls_distance/next_num_balls : 0;
      max_balls_distance = next_max_balls_distance;

      num_lines = next_num_lines;
      num_lines_per_frame = (next_num_frames > 0) ?
                           (double) next_num_lines/next_num_frames : 0;

      num_penalty_spots = next_num_penalty_spots;
      num_penalty_spots_per_frame = (next_num_frames > 0) ?
                           (double) next_num_penalty_spots/next_num_frames : 0;
      num_centre_circles = next_num_centre_circles;
      num_centre_circles_per_frame = (next_num_frames > 0) ?
                           (double) next_num_centre_circles/next_num_frames : 0;
      num_field_line_points = next_num_field_line_points;
      num_field_line_points_per_frame = (next_num_frames > 0) ?
                           (double) next_num_field_line_points/next_num_frames : 0;

      // Reset the next set of info to be taken
      next_num_frames = 0;

      next_num_corners = 0;
      next_min_corners_distance = 0;
      next_avg_corners_distance = 0;
      next_max_corners_distance = 0;
      
      next_num_t_junctions = 0;
      next_min_t_junctions_distance = 0;
      next_avg_t_junctions_distance = 0;
      next_max_t_junctions_distance = 0;

      next_num_posts = 0;
      next_min_posts_kDistance = 0;
      next_avg_posts_kDistance = 0;
      next_max_posts_kDistance = 0;

      next_min_posts_wDistance = 0;
      next_avg_posts_wDistance = 0;
      next_max_posts_wDistance = 0;

      next_num_robots = 0;

      next_num_balls = 0;
      next_min_balls_distance = 0;
      next_avg_balls_distance = 0;
      next_max_balls_distance = 0;      

      next_num_lines = 0;
      
      next_num_penalty_spots = 0;
      next_num_centre_circles = 0;
      next_num_field_line_points = 0;
   }

   template <class Archive>
   void serialize(Archive& ar, const unsigned int file_version) {
      ar & initial_timestamp;
      
      // Features collected in the last second
      ar & num_frames;

      ar & num_corners;
      ar & num_corners_per_frame;
      ar & min_corners_distance;
      ar & avg_corners_distance;
      ar & max_corners_distance;

      ar & num_t_junctions;
      ar & num_t_junctions_per_frame;
      ar & min_t_junctions_distance;
      ar & avg_t_junctions_distance;
      ar & max_t_junctions_distance;
      
      ar & num_posts;
      ar & num_posts_per_frame;
      ar & min_posts_kDistance;
      ar & avg_posts_kDistance;
      ar & max_posts_kDistance;

      ar & min_posts_wDistance;
      ar & avg_posts_wDistance;
      ar & max_posts_wDistance;

      ar & num_robots;
      ar & num_robots_per_frame;

      ar & num_balls;
      ar & num_balls_per_frame;
      ar & min_balls_distance;
      ar & avg_balls_distance;
      ar & max_balls_distance;
      
      ar & num_lines;
      ar & num_lines_per_frame;

      ar & num_penalty_spots;
      ar & num_penalty_spots_per_frame;
      ar & num_centre_circles;
      ar & num_centre_circles_per_frame;
      ar & num_field_line_points;
      ar & num_field_line_points_per_frame;
   }
};

inline std::ostream& operator<<(std::ostream& os, const LastSecondInfo& lastSecond) {
   os.write((char*) &(lastSecond.initial_timestamp), sizeof(int64_t));

   // Features currently being collected
   os.write((char*) &(lastSecond.next_num_frames), sizeof(int));
   
   os.write((char*) &(lastSecond.next_num_corners), sizeof(int));
   os.write((char*) &(lastSecond.next_min_corners_distance), sizeof(int));
   os.write((char*) &(lastSecond.next_avg_corners_distance), sizeof(int));
   os.write((char*) &(lastSecond.next_max_corners_distance), sizeof(int));

   os.write((char*) &(lastSecond.next_num_t_junctions), sizeof(int));
   os.write((char*) &(lastSecond.next_min_t_junctions_distance), sizeof(int));
   os.write((char*) &(lastSecond.next_avg_t_junctions_distance), sizeof(int));
   os.write((char*) &(lastSecond.next_max_t_junctions_distance), sizeof(int));

   os.write((char*) &(lastSecond.next_num_posts), sizeof(int));
   os.write((char*) &(lastSecond.next_min_posts_kDistance), sizeof(int));
   os.write((char*) &(lastSecond.next_avg_posts_kDistance), sizeof(int));
   os.write((char*) &(lastSecond.next_max_posts_kDistance), sizeof(int));

   os.write((char*) &(lastSecond.next_min_posts_wDistance), sizeof(int));
   os.write((char*) &(lastSecond.next_avg_posts_wDistance), sizeof(int));
   os.write((char*) &(lastSecond.next_max_posts_wDistance), sizeof(int));

   os.write((char*) &(lastSecond.next_num_robots), sizeof(int));

   os.write((char*) &(lastSecond.next_num_balls), sizeof(int));
   os.write((char*) &(lastSecond.next_min_balls_distance), sizeof(int));
   os.write((char*) &(lastSecond.next_avg_balls_distance), sizeof(int));
   os.write((char*) &(lastSecond.next_max_balls_distance), sizeof(int));
   
   os.write((char*) &(lastSecond.next_num_lines), sizeof(int));

   os.write((char*) &(lastSecond.next_num_penalty_spots), sizeof(int));
   os.write((char*) &(lastSecond.next_num_centre_circles), sizeof(int));
   os.write((char*) &(lastSecond.next_num_field_line_points), sizeof(int));

   // Features collected in the last second
   os.write((char*) &(lastSecond.num_frames), sizeof(int));

   os.write((char*) &(lastSecond.num_corners), sizeof(int));
   os.write((char*) &(lastSecond.num_corners_per_frame), sizeof(double));
   os.write((char*) &(lastSecond.min_corners_distance), sizeof(int));
   os.write((char*) &(lastSecond.avg_corners_distance), sizeof(int));
   os.write((char*) &(lastSecond.max_corners_distance), sizeof(int));
   
   os.write((char*) &(lastSecond.num_t_junctions), sizeof(int));
   os.write((char*) &(lastSecond.num_t_junctions_per_frame), sizeof(double));
   os.write((char*) &(lastSecond.min_t_junctions_distance), sizeof(int));
   os.write((char*) &(lastSecond.avg_t_junctions_distance), sizeof(int));
   os.write((char*) &(lastSecond.max_t_junctions_distance), sizeof(int));

   os.write((char*) &(lastSecond.num_posts), sizeof(int));
   os.write((char*) &(lastSecond.num_posts_per_frame), sizeof(double));
   os.write((char*) &(lastSecond.min_posts_kDistance), sizeof(int));
   os.write((char*) &(lastSecond.avg_posts_kDistance), sizeof(int));
   os.write((char*) &(lastSecond.max_posts_kDistance), sizeof(int));

   os.write((char*) &(lastSecond.min_posts_wDistance), sizeof(int));
   os.write((char*) &(lastSecond.avg_posts_wDistance), sizeof(int));
   os.write((char*) &(lastSecond.max_posts_wDistance), sizeof(int));

   os.write((char*) &(lastSecond.num_robots), sizeof(int));
   os.write((char*) &(lastSecond.num_robots_per_frame), sizeof(double));

   os.write((char*) &(lastSecond.num_balls), sizeof(int));
   os.write((char*) &(lastSecond.num_balls_per_frame), sizeof(double));
   os.write((char*) &(lastSecond.min_balls_distance), sizeof(int));
   os.write((char*) &(lastSecond.avg_balls_distance), sizeof(int));
   os.write((char*) &(lastSecond.max_balls_distance), sizeof(int));
   
   os.write((char*) &(lastSecond.num_lines), sizeof(int));
   os.write((char*) &(lastSecond.num_lines_per_frame), sizeof(double));

   os.write((char*) &(lastSecond.num_penalty_spots), sizeof(int));
   os.write((char*) &(lastSecond.num_penalty_spots_per_frame), sizeof(double));
   os.write((char*) &(lastSecond.num_centre_circles), sizeof(int));
   os.write((char*) &(lastSecond.num_centre_circles_per_frame), sizeof(double));
   os.write((char*) &(lastSecond.num_field_line_points), sizeof(int));
   os.write((char*) &(lastSecond.num_field_line_points_per_frame), sizeof(double));

   return os;
}

inline std::istream& operator>>(std::istream& is, LastSecondInfo& lastSecond) {
   is.read((char*) &(lastSecond.initial_timestamp), sizeof(int64_t));

   // Features currently being collected
   is.read((char*) &(lastSecond.next_num_frames), sizeof(int));
   
   is.read((char*) &(lastSecond.next_num_corners), sizeof(int));
   is.read((char*) &(lastSecond.next_min_corners_distance), sizeof(int));
   is.read((char*) &(lastSecond.next_avg_corners_distance), sizeof(int));
   is.read((char*) &(lastSecond.next_max_corners_distance), sizeof(int));
   
   is.read((char*) &(lastSecond.next_num_t_junctions), sizeof(int));
   is.read((char*) &(lastSecond.next_min_t_junctions_distance), sizeof(int));
   is.read((char*) &(lastSecond.next_avg_t_junctions_distance), sizeof(int));
   is.read((char*) &(lastSecond.next_max_t_junctions_distance), sizeof(int));

   is.read((char*) &(lastSecond.next_num_posts), sizeof(int));
   is.read((char*) &(lastSecond.next_min_posts_kDistance), sizeof(int));
   is.read((char*) &(lastSecond.next_avg_posts_kDistance), sizeof(int));
   is.read((char*) &(lastSecond.next_max_posts_kDistance), sizeof(int));

   is.read((char*) &(lastSecond.next_min_posts_wDistance), sizeof(int));
   is.read((char*) &(lastSecond.next_avg_posts_wDistance), sizeof(int));
   is.read((char*) &(lastSecond.next_max_posts_wDistance), sizeof(int));

   is.read((char*) &(lastSecond.next_num_robots), sizeof(int));

   is.read((char*) &(lastSecond.next_num_balls), sizeof(int));
   is.read((char*) &(lastSecond.next_min_balls_distance), sizeof(int));
   is.read((char*) &(lastSecond.next_avg_balls_distance), sizeof(int));
   is.read((char*) &(lastSecond.next_max_balls_distance), sizeof(int));

   is.read((char*) &(lastSecond.next_num_lines), sizeof(int));

   is.read((char*) &(lastSecond.next_num_penalty_spots), sizeof(int));
   is.read((char*) &(lastSecond.next_num_centre_circles), sizeof(int));
   is.read((char*) &(lastSecond.next_num_field_line_points), sizeof(int));

   // Features collected in the last second
   is.read((char*) &(lastSecond.num_frames), sizeof(int));

   is.read((char*) &(lastSecond.num_corners), sizeof(int));
   is.read((char*) &(lastSecond.num_corners_per_frame), sizeof(double));
   is.read((char*) &(lastSecond.min_corners_distance), sizeof(int));
   is.read((char*) &(lastSecond.avg_corners_distance), sizeof(int));
   is.read((char*) &(lastSecond.max_corners_distance), sizeof(int));
   
   is.read((char*) &(lastSecond.num_t_junctions), sizeof(int));
   is.read((char*) &(lastSecond.num_t_junctions_per_frame), sizeof(double));
   is.read((char*) &(lastSecond.min_t_junctions_distance), sizeof(int));
   is.read((char*) &(lastSecond.avg_t_junctions_distance), sizeof(int));
   is.read((char*) &(lastSecond.max_t_junctions_distance), sizeof(int));

   is.read((char*) &(lastSecond.num_posts), sizeof(int));
   is.read((char*) &(lastSecond.num_posts_per_frame), sizeof(double));
   is.read((char*) &(lastSecond.min_posts_kDistance), sizeof(int));
   is.read((char*) &(lastSecond.avg_posts_kDistance), sizeof(int));
   is.read((char*) &(lastSecond.max_posts_kDistance), sizeof(int));

   is.read((char*) &(lastSecond.min_posts_wDistance), sizeof(int));
   is.read((char*) &(lastSecond.avg_posts_wDistance), sizeof(int));
   is.read((char*) &(lastSecond.max_posts_wDistance), sizeof(int));

   is.read((char*) &(lastSecond.num_robots), sizeof(int));
   is.read((char*) &(lastSecond.num_robots_per_frame), sizeof(double));

   is.read((char*) &(lastSecond.num_balls), sizeof(int));
   is.read((char*) &(lastSecond.num_balls_per_frame), sizeof(double));
   is.read((char*) &(lastSecond.min_balls_distance), sizeof(int));
   is.read((char*) &(lastSecond.avg_balls_distance), sizeof(int));
   is.read((char*) &(lastSecond.max_balls_distance), sizeof(int));
   
   is.read((char*) &(lastSecond.num_lines), sizeof(int));
   is.read((char*) &(lastSecond.num_lines_per_frame), sizeof(double));

   is.read((char*) &(lastSecond.num_penalty_spots), sizeof(int));
   is.read((char*) &(lastSecond.num_penalty_spots_per_frame), sizeof(double));
   is.read((char*) &(lastSecond.num_centre_circles), sizeof(int));
   is.read((char*) &(lastSecond.num_centre_circles_per_frame), sizeof(double));
   is.read((char*) &(lastSecond.num_field_line_points), sizeof(int));
   is.read((char*) &(lastSecond.num_field_line_points_per_frame), sizeof(double));
   return is;
}
