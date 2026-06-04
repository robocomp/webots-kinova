/*
 *    Copyright (C) 2025 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

#include <array>
#include <cmath>
#include <fstream>
#include "rapplication/rapplication.h"

#pragma region ROBOCOMP_METHODS

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}

        wheelsMatrix <<
                     1.0/ WHEEL_RADIUS,  -1.0/ WHEEL_RADIUS, SumLxLyOverRadius * ROTATION_INCREMENT_COEFFICIENT,
                    -1.0/ WHEEL_RADIUS,  -1.0/ WHEEL_RADIUS,  SumLxLyOverRadius * ROTATION_INCREMENT_COEFFICIENT,
                    1.0/ WHEEL_RADIUS,   1.0/ WHEEL_RADIUS,  SumLxLyOverRadius * ROTATION_INCREMENT_COEFFICIENT,
                    -1.0/ WHEEL_RADIUS,  1.0/ WHEEL_RADIUS, SumLxLyOverRadius * ROTATION_INCREMENT_COEFFICIENT;

    }
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "Initialize worker" << std::endl;

    robot = new webots::Supervisor();
    if(!robot) 
    {
        
        std::cerr << "Error: No se pudo crear el Supervisor de Webots." << std::endl;
        return;
    }
    robotNode = robot->getSelf();


    // Base motors and sensors initialization.
    // const char *motorNames[4] = {"wheel1", "wheel2", "wheel3", "wheel4"};
    // for (int i = 0; i < 4; i++)
    // {
    //     motors[i] = robot->getMotor(motorNames[i]);
    //     positionSensors[i] = motors[i]->getPositionSensor();
    //     positionSensors[i]->enable(this->getPeriod("Compute"));
    //     motors[i]->setPosition(INFINITY); // Speed Mode
    //     motors[i]->setVelocity(0);
    // }

    // Right Kinova Arm motors and sensors initialization.
    std::string prefix = "Right_";
    
    for (size_t i = 0; i < kinovaMotorNames.size(); ++i) {

        kinovaArmRMotors.push_back(robot->getMotor(prefix + kinovaMotorNames[i]));
        kinovaArmRSensors.push_back(robot->getPositionSensor(prefix + kinovaSensorNames[i]));

        if (!kinovaArmRMotors[i] || !kinovaArmRSensors[i]) {
            std::cerr << "Error: No se pudo obtener el motor o sensor para el actuator " << i+1 << std::endl;
            continue;
        }

        kinovaArmRSensors[i]->enable(this->getPeriod("Compute"));
        // Joint torque feedback — the raw signal the controller turns into a
        // 6-axis wrist wrench via w = (Jᵀ)⁺·τ (matches how the Gen3 estimates
        // its tool F/T). Exposed per joint through TJoint.torque in getJoints().
        kinovaArmRMotors[i]->enableTorqueFeedback(this->getPeriod("Compute"));
    }

    // // Left Kinova Arm motors and sensors initialization.
    // prefix = "Left_";
    // for (size_t i = 0; i < kinovaMotorNames.size(); ++i) {

    //     kinovaArmLMotors.push_back(robot->getMotor(prefix + kinovaMotorNames[i]));
    //     kinovaArmLSensors.push_back(robot->getPositionSensor(prefix + kinovaSensorNames[i]));

    //     if (!kinovaArmLMotors[i] || !kinovaArmLSensors[i]) {
    //         std::cerr << "Error: No se pudo obtener el motor o sensor para el actuator " << i+1 << std::endl;
    //         continue;
    //     }

    //     kinovaArmLSensors[i]->enable(this->getPeriod("Compute"));
    // }

    // Camera360 initialization
    // camera360_1 = robot->getCamera("camera_360_1");
    // camera360_2 = robot->getCamera("camera_360_2");
    // if(camera360_1 && camera360_2){
    //     camera360_1->enable(this->getPeriod("Compute"));
    //     camera360_2->enable(this->getPeriod("Compute"));
    // }

    // // Zed initialization
    // zed = robot->getCamera("zed");
    // zedRangeFinder = robot->getRangeFinder("zed-ranger");
    // if(zed) zed->enable(this->getPeriod("Compute"));
    // if(zedRangeFinder) zedRangeFinder->enable(this->getPeriod("Compute"));

    // // Helios Lidar initialization
    // heliosLidar = robot->getLidar("helios");
    // if(heliosLidar) heliosLidar->enable(this->getPeriod("Compute"));

    // // Accelerometer initialization
    // accelerometer = robot->getAccelerometer("accelerometer");
    // if(accelerometer) accelerometer->enable(this->getPeriod("Compute"));

    // Gripper slider limits. These match the PROTO defaults declared in
    // protos/KinovaGen3.proto (armsMinPosition = 0, armsMaxPosition = 0.0425):
    //   .first  = fully-closed slider position (m, 0)
    //   .second = fully-open  slider position (m, 0.0425 ≈ 42.5 mm)
    // The supervisor lookup below would be the principled way to read these
    // off the live node, but it requires the arm to carry a DEF, which our
    // world doesn't currently provide. Hard-code matches PROTO defaults.
    armsMinMaxPosition.first  = 0.0f;
    armsMinMaxPosition.second = 0.0425f;
    // armsMinMaxPosition.first = robot->getFromDef("KINOVA_ARM_L")->getField("armsMinPosition")->getSFFloat();
    // armsMinMaxPosition.second = robot->getFromDef("KINOVA_ARM_L")->getField("armsMaxPosition")->getSFFloat();

    // left_hand.first = robot->getMotor("Left_Hand_LinearMotor_Right");
    // left_hand.second = robot->getMotor("Left_Hand_LinearMotor_Left");
    // /*
    // if (left_hand.first && left_hand.second) {
    //     left_hand.first->setPosition(INFINITY); // Speed Mode
    //     left_hand.first->setVelocity(0);
    //     left_hand.second->setPosition(INFINITY);
    //     left_hand.second->setVelocity(0);
    // }
    // */

    right_hand.first = robot->getMotor("Right_Hand_LinearMotor_Right");
    right_hand.second = robot->getMotor("Right_Hand_LinearMotor_Left");

    // Guard: a wrong Webots instance / missing gripper would make getMotor return
    // null and the calls below segfault silently right after "Initialize worker".
    // Fail loudly with the device name instead.
    if (not right_hand.first or not right_hand.second)
    {
        std::cerr << "FATAL: gripper LinearMotors not found "
                     "(Right_Hand_LinearMotor_Right/Left) — is the bridge bound to the "
                     "Kinova arm world? Skipping gripper init.\n";
    }
    else
    {
        right_hand.first->setAvailableForce(40.0); // Fuerza suficiente para sostener, no para romper
        right_hand.second->setAvailableForce(40.0);
        // Grip-force feedback from the finger linear motors: the force the motor
        // exerts, which spikes when a finger is blocked by a grasped object. More
        // robust than the fingertip TouchSensor, which can net to ~0 in static
        // equilibrium (the slider joint reacts the contact). For a LinearMotor,
        // getTorqueFeedback() returns the linear force in N.
        right_hand.first->enableTorqueFeedback(this->getPeriod("Compute"));
        right_hand.second->enableTorqueFeedback(this->getPeriod("Compute"));
    }

    // Fingertip force sensors (force-3d TouchSensors declared on each finger
    // box in the PROTO). Device name = the finger Solid name.
    finger_force_right = robot->getTouchSensor("Right_Hand_Finger_Right");
    finger_force_left  = robot->getTouchSensor("Right_Hand_Finger_Left");
    if (finger_force_right) finger_force_right->enable(this->getPeriod("Compute"));
    if (finger_force_left)  finger_force_left->enable(this->getPeriod("Compute"));
    if (not finger_force_right or not finger_force_left)
        std::cerr << "WARN: fingertip force sensor(s) not found — getGripperState forces will be 0\n";

    KinovaArm_setGripperPos(1);   // start fully OPEN (0=closed, 1=open) — ready to grasp
    // Heads-up if the world has baked the arm pose into link rotations (see the helper):
    // a baked world makes Webots disagree with the URDF/Pinocchio model and is the source
    // of "the pose looks wrong" confusion.
    warn_if_world_pose_baked();

    // Startup pose: drive the arm to the controller's rest pose — the last committed
    // "elbow outside, hand forward" posture, kept in sync with Controller.rest_pose in
    // kinova_controller/etc/config.toml. With an un-baked world (joints load at 0 = the
    // design pose, matching the URDF) teleport_arm_to sets the joints there instantly
    // with no swept collision; moveBothArmsWithAngle then pins the motors to hold it.
    // Decoded from the world's (now-removed) baked hidden rotations — i.e. the exact
    // pose the arm showed on reset, expressed in clean joint space. Trustworthy because
    // it was measured from the world itself, not tuned against a baked offset. Keep in
    // sync with Controller.rest_pose in kinova_controller/etc/config.toml.
    const std::array<double, 7> rest_pose = {0.30, 0.80, 1.50, -2.101, 0.651, -1.02, 3.141};
    const RoboCompKinovaArm::Angles rest_angles(rest_pose.begin(), rest_pose.end());
    teleport_arm_to(rest_angles);
    moveBothArmsWithAngle(rest_angles, kinovaArmRMotors);
    // KinovaArm1_setGripperPos(0);
    // KinovaArm1_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles{RoboCompKinovaArm::Angles{-0.698, -2.0944, -1.047, -2.2689, 0.3491, -1.1345, 1.4835}});


    /*
    if (right_hand.first && right_hand.second) {
        right_hand.first->setPosition(INFINITY); // Speed Mode
        right_hand.first->setVelocity(0);
        right_hand.second->setPosition(INFINITY);
        right_hand.second->setVelocity(0);
    }
    */
    std::cout << "Worker initialized OK" << std::endl;
}


void SpecificWorker::compute()
{
    
    //double now = robot->getTime() * 1000;

    //if(robot) receiving_robotSpeed(robot, now);
    // if(camera360_1 && camera360_2) receiving_camera360Data(camera360_1, camera360_2, now);
    // if(heliosLidar) receiving_lidarData(heliosLidar, double_buffer_helios,  helios_delay_queue, now);
    // if(zedRangeFinder && zed) receiving_cameraRGBD(zed, zedRangeFinder, zedImage, now);

    robot->step(this->getPeriod("Compute"));

    fps.print("FPS:");
}

////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
}



//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

#pragma endregion ROBOCOMP_METHODS

void SpecificWorker::receiving_robotSpeed(webots::Supervisor* _robot, double timestamp)
{
    const double* shadow_position = robotNode->getPosition();
    const double* shadow_orientation = robotNode->getOrientation();
    const double* shadow_velocity = robotNode->getVelocity();

    double r11 = shadow_orientation[0], r12 = shadow_orientation[1], r13 = shadow_orientation[2];
    double r21 = shadow_orientation[3], r22 = shadow_orientation[4], r23 = shadow_orientation[5];
    double r31 = shadow_orientation[6], r32 = shadow_orientation[7], r33 = shadow_orientation[8];

    // Yaw (Z)
    double rz = atan2(r21, r11);
    // Pitch (Y)
    double ry = atan2(-r31, sqrt(r32*r32 + r33*r33));
    // Roll (X)
    double rx = atan2(r32, r33);

    float orientation = atan2(shadow_orientation[1], shadow_orientation[0]) - M_PI_2;

    Eigen::Matrix2f rt_rotation_matrix;
    rt_rotation_matrix << cos(orientation), -sin(orientation),
            sin(orientation), cos(orientation);

    // Multiply the velocity vector by the inverse of the rotation matrix to get the velocity in the robot reference system
    Eigen::Vector2f shadow_velocity_2d(shadow_velocity[1], shadow_velocity[0]);
    Eigen::Vector2f rt_rotation_matrix_inv = rt_rotation_matrix.inverse() * shadow_velocity_2d;

    // Velocidades puras en mm/s y rad/s
    double velocidad_x = 0.1; // Ejemplo: 100 mm/s
    double velocidad_y = 0.1; // Ejemplo: 150 mm/s
    double alpha = 0.075; // Ejemplo: 0.05 rad/s

    // Desviación estándar del ruido (ejemplo: 5% del valor de las velocidades)
    double ruido_stddev_x = 0.05 * velocidad_x;
    double ruido_stddev_y = 0.05 * velocidad_y;
    double ruido_stddev_alpha = 0.05 * alpha;

    RoboCompFullPoseEstimation::FullPoseEuler pose_data;

    // Posición
    pose_data.x = -shadow_position[1] * 1000;  // metros → mm
    pose_data.y = shadow_position[0] * 1000;
    pose_data.z = shadow_position[2] * 1000;

    // Orientación (Euler en radianes) 2D
    pose_data.rx = rx;
    pose_data.ry = ry;
    pose_data.rz = rz;

    pose_data.vx = rt_rotation_matrix_inv(1) + generateNoise(ruido_stddev_y);
    pose_data.vy = -rt_rotation_matrix_inv(0) + generateNoise(ruido_stddev_x);
    pose_data.vz = 0;
    pose_data.vrx = 0;
    pose_data.vry = 0;
    pose_data.vrz = shadow_velocity[5] + generateNoise(ruido_stddev_alpha);
    pose_data.timestamp = timestamp;


    // std::cout << "pose:" << pose_data.x << " " << pose_data.y << " " << pose_data.z <<
    //                 " rot: "<< pose_data.rx << " " << pose_data.ry << " " << pose_data.rz << std::endl;

    this->fullposeestimationpub_pubproxy->newFullPose(pose_data);
}

void SpecificWorker::receiving_camera360Data(webots::Camera* _camera1, webots::Camera* _camera2, double timestamp)
{
    RoboCompCamera360RGB::TImage newImage360;

    // Aseguramos de que ambas cámaras tienen la misma resolución, de lo contrario, deberás manejar las diferencias.
    if (_camera1->getWidth() != _camera2->getWidth() || _camera1->getHeight() != _camera2->getHeight())
    {
        std::cerr << "Error: Cameras with different resolutions." << std::endl;
        return;
    }

    // Timestamp calculation
    newImage360.timestamp = timestamp;

    // La resolución de la nueva imagen será el doble en el ancho ya que estamos combinando las dos imágenes.
    newImage360.width = 2 * _camera1->getWidth();
    newImage360.height = _camera1->getHeight();

    // Establecer el periodo real del compute de refresco de la imagen en milisegundos.
    newImage360.period = fps.get_period();

    const unsigned char* webotsImageData1 = _camera1->getImage();
    const unsigned char* webotsImageData2 = _camera2->getImage();
    cv::Mat img_1 = cv::Mat(cv::Size(_camera1->getWidth(), _camera1->getHeight()), CV_8UC4);
    cv::Mat img_2 = cv::Mat(cv::Size(_camera2->getWidth(), _camera2->getHeight()), CV_8UC4);

    img_1.data = (uchar *)webotsImageData1;
    cv::cvtColor(img_1, img_1, cv::COLOR_RGBA2RGB);

    img_2.data = (uchar *)webotsImageData2;
    cv::cvtColor(img_2, img_2, cv::COLOR_RGBA2RGB);

    cv::Mat img_final = cv::Mat(cv::Size(_camera1->getWidth()*2, _camera1->getHeight()), CV_8UC3);

    img_1.copyTo(img_final(cv::Rect(0, 0, _camera1->getWidth(), _camera1->getHeight())));
    img_2.copyTo(img_final(cv::Rect(_camera1->getWidth(), 0, _camera1->getWidth(), _camera2->getHeight())));

    // Asignar la imagen RGB 360 al tipo TImage de Robocomp
    newImage360.image.resize(img_final.total()*img_final.elemSize());
    memcpy(&newImage360.image[0], img_final.data, img_final.total()*img_final.elemSize());

    //newImage360.image = rgbImage360;
    newImage360.compressed = false;

    if(pars.delay)
        camera_queue.push(newImage360);

    // Asignamo el resultado final al atributo de clase (si tienes uno).
    double_buffer_360.put(std::move(newImage360));

    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - now).count() << std::endl;
}

void SpecificWorker::receiving_lidarData(webots::Lidar* _lidar, DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData> &_lidar3dData, FixedSizeDeque<RoboCompLidar3D::TData>& delay_queue, double timestamp)
{
    if (!_lidar) { std::cout << "No lidar available." << std::endl; return; }

    const float *rangeImage = _lidar->getRangeImage();
    int horizontalResolution = _lidar->getHorizontalResolution();
    int verticalResolution = _lidar->getNumberOfLayers();
    double fov = _lidar->getFov();
    double angleResolution = fov / horizontalResolution;
    float verticalFov = 2.8;

    RoboCompLidar3D::TData newLidar3dData;

    // General Lidar values
    newLidar3dData.timestamp = timestamp;
    newLidar3dData.period = fps.get_period();

    if(!rangeImage) { std::cout << "Lidar data empty." << std::endl; return; }

    for (int j = 0; j < verticalResolution; ++j) {
        for (int i = 0; i < horizontalResolution; ++i) {
            int index = j * horizontalResolution + i;

            //distance meters to millimeters
            const float distance = rangeImage[index]; //Meters

            //TODO rotacion del eje y con el M_PI, solucionar
            float horizontalAngle = M_PI - i * angleResolution - fov / 2;

            float verticalAngle = M_PI + j * (verticalFov / verticalResolution) - verticalFov / 2; //down limit+, uper limit-, horizon line is PI

            //Calculate Cartesian co-ordinates and rectify axis positions
            Eigen::Vector3f lidar_point(
                    distance * cos(horizontalAngle) * cos(verticalAngle),
                    distance * sin(horizontalAngle) * cos(verticalAngle),
                    distance * sin(verticalAngle));

            if (not (std::isinf(lidar_point.x()) or std::isinf(lidar_point.y()) or std::isinf(lidar_point.z())))
            {
                // if (not (verticalAngle > 4.10152 or verticalAngle <2.87979)) //helios normal position
                if (not (verticalAngle > 3.40339 or verticalAngle < 2.1816)) //helios flipped position
                {
                    RoboCompLidar3D::TPoint point;

                    point.x = lidar_point.x();
                    point.y = lidar_point.y();
                    point.z = lidar_point.z();

                    point.r = lidar_point.norm();  // distancia radial
                    point.phi = horizontalAngle;  // ángulo horizontal // -x para hacer [PI, -PI] y no [-PI, PI]
                    point.theta = verticalAngle;  // ángulo vertical
                    point.distance2d = std::hypot(lidar_point.x(),lidar_point.y());  // distancia en el plano xy

                    newLidar3dData.points.push_back(point);
                }
            }
        }
    }
    //Points order to angles
    std::ranges::sort(newLidar3dData.points, {}, &RoboCompLidar3D::TPoint::phi);

    //Is it necessary to use two lidar queues? One for each lidaR?
    if(pars.delay)
        delay_queue.push(newLidar3dData);

    _lidar3dData.put(std::move(newLidar3dData));
}

double SpecificWorker::generateNoise(double stddev)
{
    std::random_device rd; // Obtiene una semilla aleatoria del hardware
    std::mt19937 gen(rd()); // Generador de números aleatorios basado en Mersenne Twister
    std::normal_distribution<> d(0, stddev); // Distribución normal con media 0 y desviación estándar stddev
    return d(gen);
}

void SpecificWorker::receiving_cameraRGBD(webots::Camera* _camera,
                                          webots::RangeFinder* _rangeFinder,
                                          RoboCompCameraRGBDSimple::TRGBD& _image,
                                          double timestamp)
{

    RoboCompCameraRGBDSimple::TRGBD new_zed_image;

    new_zed_image.image.alivetime = new_zed_image.depth.alivetime = new_zed_image.points.alivetime = timestamp;
    new_zed_image.image.period = new_zed_image.depth.period = new_zed_image.points.period = fps.get_period();

    int width = _camera->getWidth();
    int height = _camera->getHeight();

    new_zed_image.image.width = new_zed_image.depth.width = width;
    new_zed_image.image.height = new_zed_image.depth.height = height;
    new_zed_image.image.compressed = new_zed_image.depth.compressed = new_zed_image.points.compressed = false;

    // -------------------- Imagen RGB --------------------
    const unsigned char *webotsImageData = _camera->getImage();
    cv::Mat imageMatBGRA(height, width, CV_8UC4, (void*)webotsImageData);

    cv::Mat imageMatRGB;
    cv::cvtColor(imageMatBGRA, imageMatRGB, cv::COLOR_BGRA2RGB);

    new_zed_image.image.image.resize(imageMatRGB.total() * imageMatRGB.channels());
    std::memcpy(new_zed_image.image.image.data(), imageMatRGB.data, new_zed_image.image.image.size());

    // -------------------- Intrínsecas --------------------
    double fov = _rangeFinder->getFov(); // radianes
    double fx = width / (2.0 * tan(fov / 2.0));
    double fy = fx;
    new_zed_image.depth.focalx = fx;
    new_zed_image.depth.focaly = fy;

    // -------------------- Imagen de profundidad --------------------
    const float* depthImage = _rangeFinder->getRangeImage();

    cv::Mat depthMat(height, width, CV_32FC1, (void*)depthImage);

    new_zed_image.depth.depth.resize(width * height * sizeof(float));
    std::memcpy(new_zed_image.depth.depth.data(), depthMat.data, new_zed_image.depth.depth.size());

    double_buffer_zed.put(std::move(new_zed_image));
}

#pragma region OMNIROBOT_INTERFACE

void SpecificWorker::OmniRobot_correctOdometer(int x, int z, float alpha)
{
    printNotImplementedWarningMessage("OmniRobot_correctOdometer");
}

void SpecificWorker::OmniRobot_getBasePose(int &x, int &z, float &alpha)
{
    const double* shadow_position = robotNode->getPosition();
    const double* shadow_orientation = robotNode->getOrientation();

    x = round(shadow_position[0]);
    z = round(shadow_position[1]);

    alpha = (float)atan2(shadow_orientation[0], shadow_orientation[3]);
    // std::cout << "X: " << x << " Z: " << z << " Alpha: " << alpha << std::endl;


}

void SpecificWorker::OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
    const double* shadow_position = robotNode->getPosition();
    const double* shadow_orientation = robotNode->getOrientation();

    state.x = (float)shadow_position[0];
    state.z = (float)shadow_position[1];

    state.alpha = (float)atan2(shadow_orientation[0], shadow_orientation[3]);

    state.correctedX = state.x;
    state.correctedZ = state.z;
    state.correctedAlpha = state.alpha;

    const double* velocity = robotNode->getVelocity(); // Devuelve [vx, vy, vz, wx, wy, wz]
    state.advVx = (float)velocity[0];
    state.advVz = (float)velocity[2];
    state.rotV = (float)velocity[5]; // Velocidad angular en Y

    state.isMoving = (abs(state.advVx) > 0.001f || abs(state.advVz) > 0.001f || abs(state.rotV) > 0.001f);

    // Debug
    //std::cout << "X: " << state.x << " Z: " << state.z << " Alpha: " << state.alpha << std::endl;
}
void SpecificWorker::OmniRobot_resetOdometer()
{
    printNotImplementedWarningMessage("OmniRobot_resetOdometer");
}

void SpecificWorker::OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state)
{
    printNotImplementedWarningMessage("OmniRobot_setOdometer");
}

void SpecificWorker::OmniRobot_setOdometerPose(int x, int z, float alpha)
{
    printNotImplementedWarningMessage("OmniRobot_setOdometerPose");
}

void SpecificWorker::OmniRobot_setSpeedBase(float advx, float advz, float rot)
{
    advz *= 0.001;
    advx *= 0.001;

    Eigen::Vector3d input_speeds(advz, advx, rot);
    Eigen::Vector4d wheel_speeds = wheelsMatrix * input_speeds;

    // std::cout << "wheelsMatrix:\n" << wheelsMatrix << std::endl;
    // std::cout << "Input speeds: [" << advz << ", " << advx << ", " << rot << "]\n";
    // std::cout << "Computed wheel speeds:\n" << wheel_speeds.transpose() << std::endl;

    for (int i = 0; i < 4; i++)
    {
        motors[i]->setVelocity(wheel_speeds[i]);
    }
}

void SpecificWorker::OmniRobot_stopBase()
{
    for (int i = 0; i < 4; i++)
    {
        motors[i]->setVelocity(0);
    }
}

#pragma endregion OMNIROBOT_INTERFACE

#pragma region KINOVA_ARM_R_INTERFACE

bool SpecificWorker::KinovaArm_closeGripper()
{
	bool ret{};
	//implementCODE

	return ret;
}

RoboCompKinovaArm::TPose SpecificWorker::KinovaArm_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo)
{
	RoboCompKinovaArm::TPose ret{};
	//implementCODE

	return ret;
}

RoboCompKinovaArm::TGripper SpecificWorker::KinovaArm_getGripperState()
{
	RoboCompKinovaArm::TGripper ret{};
	// |force| from each fingertip force-3d TouchSensor (N). With one sensor per
	// finger the finger and tip forces are the same value.
	const auto force_mag = [](webots::TouchSensor* ts) -> float
	{
		if (not ts) return 0.0f;
		const double* v = ts->getValues();              // {Fx, Fy, Fz}
		if (not v) return 0.0f;
		return static_cast<float>(std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]));
	};
	// Tactile/tip force from the fingertip TouchSensors (real-robot-like).
	const float fr_tip = force_mag(finger_force_right);
	const float fl_tip = force_mag(finger_force_left);
	// Grip force from the finger motor force feedback (robust contact signal).
	const float fr_mot = right_hand.first  ? std::abs(static_cast<float>(right_hand.first->getTorqueFeedback()))  : 0.0f;
	const float fl_mot = right_hand.second ? std::abs(static_cast<float>(right_hand.second->getTorqueFeedback())) : 0.0f;
	ret.rforce    = fr_mot;   ret.lforce    = fl_mot;   // grip force (motor)
	ret.rtipforce = fr_tip;   ret.ltipforce = fl_tip;   // tip force (tactile)

	// Diagnostic: compare both sources, throttled.
	static int dbg = 0;
	if (dbg++ % 20 == 0)
		std::cout << "[gripper-force] tip(touch) R=" << fr_tip << " L=" << fl_tip
		          << "  grip(motor) R=" << fr_mot << " L=" << fl_mot << std::endl;
	return ret;
}

RoboCompKinovaArm::TJoints SpecificWorker::KinovaArm_getJointsState()
{
    return getJoints(kinovaArmRSensors, kinovaArmRMotors);
}

RoboCompKinovaArm::TToolInfo SpecificWorker::KinovaArm_getToolInfo()
{
	RoboCompKinovaArm::TToolInfo ret{};
	//implementCODE

	return ret;
}

void SpecificWorker::KinovaArm_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles)
{
    moveBothArmsWithAngle(angles.jointAngles, kinovaArmRMotors);
}

void SpecificWorker::KinovaArm_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds)
{
    moveBothArmsWithSpeed(speeds.jointSpeeds, kinovaArmRMotors);
}
void SpecificWorker::KinovaArm_openGripper()
{
	//implementCODE

}

void SpecificWorker::KinovaArm_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo)
{
	//implementCODE

}

bool SpecificWorker::KinovaArm_setGripperPos(float pos)
{
    // Convention (matches the KinovaArm.idsl contract used by the
    // active-inference controller): pos ∈ [0, 1] with
    //   pos = 0 → fully closed (slider at armsMinPosition)
    //   pos = 1 → fully open   (slider at armsMaxPosition).
    // Earlier code inverted this — that produced "1 = closed" silently
    // because armsMinMaxPosition was zero-initialised and the inversion
    // hid the bug; now both are fixed together.
    if (not right_hand.first or not right_hand.second)
        return false;                 // gripper motors absent (see initialize guard)
    pos = std::clamp(pos, 0.0f, 1.0f);
    float target = armsMinMaxPosition.first + pos * (armsMinMaxPosition.second - armsMinMaxPosition.first);     // [0,1] → [minPosition, maxPosition]

    // Pull the commanded position 1 mm inside the slider's hard limits.
    // Commanding exactly at minStop or maxStop makes the motor integrate
    // error against the joint constraint, producing high-frequency shiver
    // that no amount of damping fully removes.
    const float margin = 0.001f;
    target = std::clamp(target,
                        armsMinMaxPosition.first  + margin,
                        armsMinMaxPosition.second - margin);

    right_hand.first->setPosition(target);
    right_hand.second->setPosition(target);

    // One-shot debug: prove the call lands and report the target written
    // to both motors. Avoids spamming at the cyclic rate while still being
    // visible on stdout the first time the controller asks.
    static bool first_setgripper_logged = false;
    if (not first_setgripper_logged)
    {
        std::cout << "[gripper] first setGripperPos call: pos_in_after_clamp="
                  << pos << "  target_slider=" << target
                  << "  range=[" << armsMinMaxPosition.first
                  << ", " << armsMinMaxPosition.second << "]" << std::endl;
        first_setgripper_logged = true;
    }

    return true;
}

#pragma endregion KINOVA_ARM_R_INTERFACE

#pragma region KINOVA_ARM_L_INTERFACE

bool SpecificWorker::KinovaArm1_closeGripper()
{
	bool ret{};
	//implementCODE

	return ret;
}

RoboCompKinovaArm::TPose SpecificWorker::KinovaArm1_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo)
{
	RoboCompKinovaArm::TPose ret{};
	//implementCODE

	return ret;
}

RoboCompKinovaArm::TGripper SpecificWorker::KinovaArm1_getGripperState()
{
	RoboCompKinovaArm::TGripper ret{};
	//implementCODE

	return ret;
}

RoboCompKinovaArm::TJoints SpecificWorker::KinovaArm1_getJointsState()
{
    return getJoints(kinovaArmLSensors, kinovaArmLMotors);
}

RoboCompKinovaArm::TToolInfo SpecificWorker::KinovaArm1_getToolInfo()
{
	RoboCompKinovaArm::TToolInfo ret{};
	//implementCODE

	return ret;
}

void SpecificWorker::KinovaArm1_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles)
{
    moveBothArmsWithAngle(angles.jointAngles, kinovaArmLMotors);
}

void SpecificWorker::KinovaArm1_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds)
{
    moveBothArmsWithSpeed(speeds.jointSpeeds, kinovaArmLMotors);
}

void SpecificWorker::KinovaArm1_openGripper()
{
	//implementCODE

}

void SpecificWorker::KinovaArm1_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo)
{
	//implementCODE

}

bool SpecificWorker::KinovaArm1_setGripperPos(float pos)
{
    pos = 1 - std::clamp(pos, 0.0f, 1.0f);


    float target = armsMinMaxPosition.first + pos * (armsMinMaxPosition.second - armsMinMaxPosition.first);     // Convertimos de [0,1] a [minPosition,maxPosition]
    // cout << target << endl;
    left_hand.first->setPosition(target);
    left_hand.second->setPosition(target);

    return true;
}

#pragma endregion KINOVA_ARM_L_INTERFACE

#pragma region CAMERA360RGB_INTERFACE

RoboCompCamera360RGB::TImage SpecificWorker::Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)
{
    if(pars.delay)
    {
        if(camera_queue.full())
            return camera_queue.back();
    }

    return double_buffer_360.get_idemp();
}

#pragma endregion CAMERA360RGB_INTERFACE

#pragma region LIDAR3D_INTERFACE

RoboCompLidar3D::TColorCloudData SpecificWorker::Lidar3D_getColorCloudData()
{
	RoboCompLidar3D::TColorCloudData ret{};
	//implementCODE

	return ret;
}
RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor)
{
    return (pars.delay && helios_delay_queue.full()) ? helios_delay_queue.back() : double_buffer_helios.get_idemp();
}

RoboCompLidar3D::TDataImage SpecificWorker::Lidar3D_getLidarDataArrayProyectedInImage(std::string name)
{
    RoboCompLidar3D::TDataImage ret{};
    printNotImplementedWarningMessage("Lidar3D_getLidarDataArrayProyectedInImage");
    return ret;
}

RoboCompLidar3D::TDataCategory SpecificWorker::Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp)
{
    RoboCompLidar3D::TDataCategory ret{};
    printNotImplementedWarningMessage("Lidar3D_getLidarDataByCategory");
    return ret;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataProyectedInImage(std::string name)
{
	RoboCompLidar3D::TData ret{};
    printNotImplementedWarningMessage("Lidar3D_getLidarDataProyectedInImage");
	return ret;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor)
{
	RoboCompLidar3D::TData ret{};
    printNotImplementedWarningMessage("Lidar3D_getLidarDataWithThreshold2d");
	return ret;
}

#pragma endregion LIDAR3D_INTERFACE


void SpecificWorker::moveBothArmsWithAngle(const RoboCompKinovaArm::Angles &jointAngles,
                                         std::vector<webots::Motor *> &armMotors) 
{
    const size_t loop_limit = std::min(jointAngles.size(), armMotors.size());
    
    #pragma omp simd
    for (size_t i = 0; i < loop_limit; ++i)
    {
        if (armMotors[i])
        {
            // Webots position-control mode: setVelocity here is the MAX
            // speed at which the motor moves toward the target. 0.25 rad/s
            // was too slow — the homing didn't finish before the external
            // controller connected and switched motors to velocity mode,
            // leaving the arm in a half-homed pose. 0.8 rad/s reaches a
            // ±π/2 target in about 2 s and stays under the Kinova Gen3
            // proto's per-motor maxVelocity of 0.8727 rad/s, so Webots
            // does not have to clip+warn every call.
            constexpr float velocity = 0.8f;

            armMotors[i]->setPosition(jointAngles[i]);
            armMotors[i]->setVelocity(velocity);
        }
        else
        {
            {
                std::cerr << "Motor nulo en la articulación " << i << std::endl;
            }
        }
    }
}

void SpecificWorker::warn_if_world_pose_baked()
{
    // Saving a world in Webots with the arm posed bakes the pose into the per-link
    // "hidden rotation_N" fields and ZEROES the joint coordinates. The joint sensors
    // then read ~0 while the arm looks posed, so Webots no longer matches the URDF /
    // Pinocchio model the controller uses (q=0 here != q=0 in the URDF) — every
    // joint-space pose then looks offset. Scan the live .wbt and warn loudly so the
    // user un-bakes it (remove hidden rotation_1..7 / translation_1..7) rather than
    // chasing a phantom calibration error. Gripper sliders (rotation_8/9) are exempt.
    const std::string world_path = robot->getWorldPath();
    std::ifstream wf(world_path);
    if (not wf)
        return;
    int baked = 0;
    std::string line;
    while (std::getline(wf, line))
    {
        const auto p = line.find("hidden rotation_");
        if (p == std::string::npos)
            continue;
        const char j = line[p + std::string("hidden rotation_").size()];
        if (j >= '1' and j <= '7')   // arm joints only
            ++baked;
    }
    if (baked > 0)
        std::cerr << "\n*** WARN: world '" << world_path << "' has " << baked
                  << " baked arm 'hidden rotation_*' field(s). ***\n"
                     "    The arm pose was saved into link rotations with the joint angles\n"
                     "    zeroed, so Webots will NOT match the URDF/Pinocchio model: joint\n"
                     "    sensors read ~0 while the arm looks posed, and every commanded\n"
                     "    pose comes out offset. Remove the hidden rotation_1..7 /\n"
                     "    translation_1..7 lines from the .wbt (do not re-save it posed).\n\n";
}

void SpecificWorker::teleport_arm_to(const RoboCompKinovaArm::Angles &jointAngles)
{
    // Set each arm joint coordinate directly through the Supervisor. Unlike
    // moveBothArmsWithAngle (which drives the motors and sweeps a path),
    // setJointPosition snaps the joint with NO dynamics, so it cannot collide on
    // the way and unwinds wound-up continuous joints instantly. getFromDevice
    // gives the motor's Node; its parent is the HingeJoint that setJointPosition
    // acts on.
    const size_t loop_limit = std::min(jointAngles.size(), kinovaArmRMotors.size());
    for (size_t i = 0; i < loop_limit; ++i)
    {
        if (not kinovaArmRMotors[i])
            continue;
        webots::Node *motor_node = robot->getFromDevice(kinovaArmRMotors[i]);
        webots::Node *joint_node = motor_node ? motor_node->getParentNode() : nullptr;
        if (joint_node)
            joint_node->setJointPosition(jointAngles[i]);
        else
            std::cerr << "WARN: could not reach HingeJoint for arm motor " << i
                      << " — that joint will not teleport\n";
    }
}

void SpecificWorker::moveBothArmsWithSpeed(const RoboCompKinovaArm::Speeds &jointSpeeds,
                                         std::vector<webots::Motor *> &armMotors) 
{
    constexpr size_t joint_limit = 7; // Asumiendo que 7 es el máximo de articulaciones
    const size_t loop_limit = std::min(jointSpeeds.size(), joint_limit);

    #pragma omp simd
    for (size_t i = 0; i < loop_limit; ++i)
    {
        if (armMotors[i])
        {
            // Desactiva el control de posición y establece velocidad
            armMotors[i]->setPosition(INFINITY);
            armMotors[i]->setVelocity(jointSpeeds[i]);
        }
        else
        {
            {
                std::cerr << "Motor nulo en la articulación " << i << std::endl;
            }
        }
    }
    
}

RoboCompKinovaArm::TJoints SpecificWorker::getJoints(std::vector<webots::PositionSensor *> &armSensors, 
                                                    std::vector<webots::Motor *> &armMotors) 
{
    constexpr size_t joint_limit = 7; 
    RoboCompKinovaArm::TJoints ret;
    ret.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch()).count();
    
    ret.joints.resize(joint_limit); // Pre-reserva espacio para 7 articulaciones
    
    #pragma omp simd
    for (int i = 0; i < joint_limit; ++i)
    {
        RoboCompKinovaArm::TJoint joint;
        joint.id = i;
        
        if (armSensors[i] && armMotors[i])
        {
            joint.angle = armSensors[i]->getValue();
            joint.velocity = armMotors[i]->getVelocity();
            joint.torque = static_cast<float>(armMotors[i]->getTorqueFeedback());
        }
        else
        {
            joint.angle = 0.0f;
            joint.velocity = 0.0f;
            joint.torque = 0.0f;

            {
                std::cerr << "Sensor o motor nulo en la articulación " << i << std::endl;
            }
        }

        // Valores por defecto
        joint.current = 0.0f;
        joint.voltage = 0.0f;
        joint.motorTemperature = 0.0f;
        joint.coreTemperature = 0.0f;
        
        ret.joints[i] = std::move(joint); // Usamos move semantics
    }
    
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::high_resolution_clock::now() - 
               std::chrono::time_point<std::chrono::high_resolution_clock>(
               std::chrono::milliseconds(ret.timestamp))).count();
    
    // std::cout << "get " << diff << " ms" << std::endl << std::flush;
    
    return ret;
}

void SpecificWorker::printNotImplementedWarningMessage(const string functionName) {
    cout << "Function not implemented used: " << "[" << functionName << "]" << std::endl;
}

#pragma region CameraRGBDSimple

RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
	return double_buffer_zed.get_idemp();
}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
	return double_buffer_zed.get_idemp().depth;
}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
	return double_buffer_zed.get_idemp().image;
}

RoboCompCameraRGBDSimple::TPoints SpecificWorker::CameraRGBDSimple_getPoints(std::string camera)
{
	return double_buffer_zed.get_idemp().points;
}

#pragma endregion CameraRGBDSimple


#pragma region JoystickAdapter

//SUBSCRIPTION to sendData method from JoystickAdapter interface
void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
#ifdef HIBERNATION_ENABLED
    hibernation = true;
#endif

    // // Declaration of the structure to be filled
    // float side=0, adv=0, rot=0;
    // /*
    // // Iterate through the list of buttons in the data structure
    // for (RoboCompJoystickAdapter::ButtonParams button : data.buttons) {
    //     // Currently does nothing with the buttons
    // }
    // */

    // // Iterate through the list of axes in the data structure
    // for (RoboCompJoystickAdapter::AxisParams axis : data.axes)
    // {
    //     // Process the axis according to its name
    //     if(axis.name == "rotate")
    //         rot = axis.value;
    //     else if (axis.name == "advance")
    //         adv = axis.value;
    //     else if (axis.name == "side")
    //         side = -axis.value;
    //     else
    //         cout << "[ JoystickAdapter ] Warning: Using a non-defined axes (" << axis.name << ")." << endl;
    // }
    // if(pars.do_joystick)
    //     OmniRobot_setSpeedBase(side, adv, rot);
}

#pragma endregion JoystickAdapter

#pragma region Webots2Robocomp

static RoboCompWebots2Robocomp::Quaternion axisAngleToQuaternion(const webots::Field &rotation)
{
    RoboCompWebots2Robocomp::Quaternion q;
    double rx = rotation.getSFRotation()[0];
    double ry = rotation.getSFRotation()[1];
    double rz = rotation.getSFRotation()[2];
    const double angle = rotation.getSFRotation()[3];

    if (const double norm = std::sqrt(rx*rx + ry*ry + rz*rz); norm > 0) {
        rx /= norm; ry /= norm; rz /= norm;
    }
    const double halfAngle = angle * 0.5;
    const double sinHalf   = std::sin(halfAngle);
    q.w = std::cos(halfAngle);
    q.x = rx * sinHalf;
    q.y = ry * sinHalf;
    q.z = rz * sinHalf;
    return q;
}

// Inverse of axisAngleToQuaternion: a unit quaternion → Webots SFRotation
// {ax, ay, az, angle}. Returns the identity axis (0,1,0) at angle 0 when the
// quaternion has no rotation, so setSFRotation always gets a valid unit axis.
static std::array<double, 4> quaternionToAxisAngle(const RoboCompWebots2Robocomp::Quaternion &q)
{
    double w = q.w, x = q.x, y = q.y, z = q.z;
    if (const double n = std::sqrt(w*w + x*x + y*y + z*z); n > 1e-9)
    { w /= n; x /= n; y /= n; z /= n; }
    const double angle = 2.0 * std::acos(std::clamp(w, -1.0, 1.0));
    const double s = std::sqrt(std::max(0.0, 1.0 - w*w));
    if (s < 1e-6) return {0.0, 1.0, 0.0, 0.0};   // ~no rotation → arbitrary unit axis
    return {x/s, y/s, z/s, angle};
}

webots::Node* SpecificWorker::find_scene_node(const std::string& key)
{
    if (auto it = scene_node_cache_.find(key); it != scene_node_cache_.end())
        return it->second;

    // 1) Try a DEF match (cheap and unambiguous when authors set DEF).
    webots::Node* node = robot->getFromDef(key);

    // 2) Fall back to a scan of the world root's top-level "children" looking
    //    for a node whose SFString "name" field equals key. Webots PROTOs
    //    (Desk, WaterBottle, …) expose a "name" but no automatic DEF, so this
    //    catches the common "I set name but forgot DEF" case.
    if (node == nullptr) {
        if (webots::Field* children = robot->getRoot()->getField("children"))
        {
            const int count = children->getCount();
            for (int i = 0; i < count; ++i)
            {
                webots::Node* child = children->getMFNode(i);
                if (child == nullptr) continue;
                webots::Field* name_field = child->getField("name");
                if (name_field == nullptr) continue;
                if (name_field->getSFString() == key) { node = child; break; }
            }
        }
    }

    if (node != nullptr) scene_node_cache_[key] = node;  // cache hits only
    return node;
}

RoboCompWebots2Robocomp::ObjectPose SpecificWorker::Webots2Robocomp_getObjectPose(std::string DEF)
{
    RoboCompWebots2Robocomp::ObjectPose ret{};
    webots::Node *object_node = find_scene_node(DEF);
    if (object_node == nullptr) {
        std::cout << "Object with DEF or name '" << DEF << "' not found" << std::endl;
        return ret;
    }
    auto object_position = object_node->getField("translation");
    auto object_rotation = object_node->getField("rotation");
    if (object_position == nullptr or object_rotation == nullptr) {
        std::cout << "Object node does not have position or rotation" << std::endl;
        return ret;
    }
    ret.position = RoboCompWebots2Robocomp::Vector3{
        static_cast<float>(object_position->getSFVec3f()[0] * 1000),
        static_cast<float>(object_position->getSFVec3f()[1] * 1000),
        static_cast<float>(object_position->getSFVec3f()[2] * 1000),
    };
    ret.orientation = axisAngleToQuaternion(*object_rotation);
    return ret;
}

void SpecificWorker::Webots2Robocomp_setObjectPose(std::string DEF, RoboCompWebots2Robocomp::ObjectPose pose)
{
    // Teleport a supervised scene node to a new pose (mm + quaternion, matching
    // getObjectPose's convention). Used to re-stand a toppled/fallen object so
    // experiments continue without a manual world reset. resetPhysics() is
    // essential: it zeroes the body's linear/angular velocity so a re-placed
    // object settles instead of carrying its fall velocity.
    webots::Node *object_node = find_scene_node(DEF);
    if (object_node == nullptr) {
        std::cout << "setObjectPose: object '" << DEF << "' not found" << std::endl;
        return;
    }
    webots::Field *t = object_node->getField("translation");
    webots::Field *r = object_node->getField("rotation");
    if (t == nullptr or r == nullptr) {
        std::cout << "setObjectPose: '" << DEF << "' has no translation/rotation" << std::endl;
        return;
    }
    const double xyz[3] = {                       // mm → m
        pose.position.x / 1000.0,
        pose.position.y / 1000.0,
        pose.position.z / 1000.0,
    };
    t->setSFVec3f(xyz);
    const auto aa = quaternionToAxisAngle(pose.orientation);
    const double rot[4] = {aa[0], aa[1], aa[2], aa[3]};
    r->setSFRotation(rot);
    object_node->resetPhysics();                  // clear residual velocity
}

void SpecificWorker::Webots2Robocomp_resetWebots()
{
    // p3bot scene has no reset hook yet.
}

void SpecificWorker::Webots2Robocomp_setDoorAngle(float angle)
{
    // p3bot scene has no door actuator.
    (void) angle;
}

void SpecificWorker::Webots2Robocomp_setPathToHuman(int humanId, RoboCompGridder::TPath path)
{
    // p3bot scene has no humans-as-supervised-nodes yet.
    (void) humanId; (void) path;
}

void SpecificWorker::Webots2Robocomp_setArmJointsInstant(RoboCompKinovaArm::TJointAngles angles)
{
    // Sim-only recovery teleport (no hardware analog — that is why it lives on the
    // supervisor interface, not KinovaArm). A controller running a long unattended
    // round calls this when it detects the arm jammed, to snap it back to a safe
    // pose WITHOUT sweeping the gripper through the table. teleport_arm_to sets the
    // joint coordinates directly (no dynamics); then pin the motors at the same
    // angles so they hold it once the controller's velocity commands stop.
    teleport_arm_to(angles.jointAngles);
    moveBothArmsWithAngle(angles.jointAngles, kinovaArmRMotors);
    std::cout << "[recovery] setArmJointsInstant: arm teleported to ["
              << angles.jointAngles.size() << " joints]" << std::endl;
}

#pragma endregion Webots2Robocomp

/**************************************/
// From the RoboCompFullPoseEstimationPub you can publish calling this methods:
// RoboCompFullPoseEstimationPub::void this->fullposeestimationpub_pubproxy->newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)

/**************************************/
// From the RoboCompCamera360RGB you can use this types:
// RoboCompCamera360RGB::TRoi
// RoboCompCamera360RGB::TImage

/**************************************/
// From the RoboCompKinovaArm you can use this types:
// RoboCompKinovaArm::TPose
// RoboCompKinovaArm::TAxis
// RoboCompKinovaArm::TToolInfo
// RoboCompKinovaArm::TGripper
// RoboCompKinovaArm::TJoint
// RoboCompKinovaArm::TJoints
// RoboCompKinovaArm::TJointSpeeds
// RoboCompKinovaArm::TJointAngles

/**************************************/
// From the RoboCompKinovaArm you can use this types:
// RoboCompKinovaArm::TPose
// RoboCompKinovaArm::TAxis
// RoboCompKinovaArm::TToolInfo
// RoboCompKinovaArm::TGripper
// RoboCompKinovaArm::TJoint
// RoboCompKinovaArm::TJoints
// RoboCompKinovaArm::TJointSpeeds
// RoboCompKinovaArm::TJointAngles

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompJoystickAdapter you can use this types:
// RoboCompJoystickAdapter::AxisParams
// RoboCompJoystickAdapter::ButtonParams
// RoboCompJoystickAdapter::TData

