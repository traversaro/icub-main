# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# depth2kin.thrift

struct Vector { }
(
   yarp.name="yarp::sig::Vector"
   yarp.includefile="yarp/sig/Vector.h"
)

struct Property { }
(
   yarp.name="yarp::os::Property"
   yarp.includefile="yarp/os/Property.h"
)

/**
* PointReq
*
* IDL structure to send/receive points.
*/
struct PointReq
{
   /**
   * contain [ok]/[fail] on success/failure.
   */
   1:string result;

   /**
   * the x-coordinate.
   */
   2:double x;

   /**
   * the y-coordinate.
   */
   3:double y;

   /**
   * the z-coordinate.
   */
   4:double z;
}

/**
* depth2kin_IDLServer
*
* IDL Interface to \ref depth2kin services.
*/
service depth2kin_IDLServer
{
   /**
   * Return the number of available experts.
   * @return the number of available experts.
   */
   i32 getNumExperts();

   /**
   * Clear the list of currently available experts.
   * @return true/false on success/failure.
   */
   bool clearExperts();

   /**
   * Reload the list of experts stored within the
   * configuration file.
   * @return true/false on success/failure.
   */
   bool load();

   /**
   * Save the current list of experts into the
   * configuration file.
   * @return true/false on success/failure.
   */
   bool save();

   /**
   * Store on file the log of system response computed
   * out of the explored set of input-output pairs.
   * @param type can be "experts" or "calibrator", accounting either
   * for the response of mixture of available experts or the output
   * of the current calibrator, respectively.
   * @return true/false on success/failure. It returns false also if
   * "calibrator" is selected and calibration has not been performed yet.
   *
   * @note Each row of the file will contain the following data: \n
   * \f$ d_x d_y d_z k_x k_y k_z r_x r_y r_z e, \f$ where \f$ d \f$
   * is the depth point, \f$ k \f$ is the kinematic point, \f$ r \f$
   * is the system response and \f$ e=|k-r| \f$ is the error.
   */
   bool log(1:string type);

   /**
   * Start the exploration phase.
   * @return true/false on success/failure.
   */
   bool explore();

   /**
   * Yield an asynchronous stop of the exploration phase.
   * @return true/false on success/failure.
   */
   bool stop();

   /**
   * Set the vergence angle used to keep the gaze fixed.
   * @param block_eyes the value in degrees of the vergence.
   * @return true/false on success/failure.
   */
   bool setBlockEyes(1:double block_eyes);

   /**
   * Return the current angle to keep the vergence at.
   * @return the vergence angle.
   */
   double getBlockEyes();

   /**
   * Tell the gaze to immediately steer the eyes to the stored
   * vergence angle and stay still.
   * @return true/false on success/failure.
   */
   bool blockEyes();

   /**
   * Select the arm to deal with.
   * @param arm is "left" or "right".
   * @return true/false on success/failure.
   */
   bool selectArm(1:string arm);

   /**
   * Set up the calibrator type.
   * @param type can be one of the following: \n
   * "se3", "se3+scale", "affine", "lssvm".
   * @param extrapolation specifies whether the calibrator will be
   * used for extrapolating data ("true") or not ("false"); if "auto"
   * is provided, then automatic choice is taken depending on the type.
   * @return true/false on success/failure.
   */
   bool setCalibrationType(1:string type, 2:string extrapolation="auto");

   /**
   * Return the current calibration type.
   * @return the calibration type.
   */
   string getCalibrationType();

   /**
   * Ask the current calibrator to carry out the calibration.
   * @return a property containing the output in terms of
   * calibration errors for each subsystem: "calibrator", "alignerL", "alignerR".
   */
   Property calibrate();

   /**
   * Push the current calibrator in the list of experts.
   * @return true/false on success/failure.
   *
   * @note the calibrator needs to have been calibrated at least once.
   */
   bool pushCalibrator();

   /**
   * Enable/disable the use of experts for touch test.
   * @param switch is "on"/"off" to use/not-use the experts.
   * @return true/false on success/failure.
   */
   bool setTouchWithExperts(1:string sw);

   /**
   * Return the current status of the switch for experts usage
   * during touch test.
   * @return "on"/"off" if experts are used/not-used.
   */
   string getTouchWithExperts();

   /**
   * Yield a <i>touch</i> action with the finger on a stereo point.
   * @param u the u-coordinate of the stereo point in the image plane.
   * @param v the v-coordinate of the stereo point in the image plane.
   * @return true/false on success/failure.
   */
   bool touch(1:i32 u, 2:i32 v);

   /**
   * Retrieve the compensated kinematic point corresponding to the input
   * depth point.
   * @param arm accounts for "left" or "right" list of experts.
   * @param x the x-coordinate of the depth point.
   * @param y the y-coordinate of the depth point.
   * @param z the z-coordinate of the depth point.
   * @return the requested point in \ref PointReq format.
   */
   PointReq getPoint(1:string arm, 2:double x, 3:double y, 4:double z);

   /**
   * Set on/off an experiment.
   * @param exp the experiment ("depth2kin" or "aligneyes") to switch on/off.
   * @param v is "on" or "off".
   * @return true/false on success/failure.
   */
   bool setExperiment(1:string exp, 2:string v);

   /**
   * Return the current status of the experiment.
   * @param exp the experiment ("depth2kin" or "aligneyes")
   * @return "on"/"off".
   */
   string getExperiment(1:string exp);

   /**
   * Retrieve the current extrinsics camera parameters.
   * @param eye is "left" or "right" camera eye.
   * @return a 6x1 vector containing the translational and the
   * rotational (in roll-pith-yaw convention) parts of the
   * extrinsics matrix.
   */
   Vector getExtrinsics(1:string eye);

   /**
   * Reset the extrinsics matrix to default eye matrix.
   * @param eye is "left" or "right" camera eye.
   * @return true/false on success/failure.
   */
   bool resetExtrinsics(1:string eye);

   /**
   * Set up the internally coded exploration space composed by
   * two co-centered ellipses, one orthogonal to other, and defined
   * by means of the center and the two semi-axes.
   * @param cx the center x-coordinate.
   * @param cy the center y-coordinate.
   * @param cz the center z-coordiante.
   * @param a the major semi-axis lenght.
   * @param b the minor semi-axis lenght.
   * @return true/false on success/failure.
   */
   bool setExplorationSpace(1:double cx, 2:double cy, 3:double cz, 4:double a, 5:double b);

   /**
   * Set up the exploration space in terms of differences with respect
   * to the internally coded couple of ellipses. 
   * @param dcx the center delta x-coordinate.
   * @param dcy the center delta y-coordinate.
   * @param dcz the center delta z-coordiante.
   * @param da the major semi-axis delta lenght.
   * @param db the minor semi-axis delta lenght.
   * @return true/false on success/failure.
   */
   bool setExplorationSpaceDelta(1:double dcx=0.0, 2:double dcy=0.0, 3:double dcz=0.0,
                                 4:double da=0.0, 5:double db=0.0);

   /**
   * Return some progress about the ongoing exploration.
   * @return a property that looks like
   * ("status" ["idle"|"ongoing"]) ("total_points" <int>) ("remaining_points" <int>)
   * ("calibrator_points" <int>) ("alignerL_points" <int>) ("alignerR_points" <int>)
   */
   Property getExplorationData();

   /**
   * Clean up the internal list of explored points pairs.
   * @return true/false on success/failure.
   */
   bool clearExplorationData();

   /**
   * Quit the module.
   * @return true/false on success/failure.
   */
   bool quit();  
}
