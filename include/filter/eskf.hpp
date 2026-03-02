#ifndef __ESKF_HPP__
#define __ESKF_HPP__
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include <eigen3/Eigen/Geometry> 
#include <fstream>

#define YEL "\033[33m"
#define RST  "\033[0m"    

class ESKF
{
public:
	
	// Default constructor
	// Input: 
	// - prediction period in seconds
	// - initial calibration time in seconds

	ESKF()
	{
		init = false;
		sec = 0;

        odom_file.open("predict_data.csv", std::ios::out);
        if (!odom_file.is_open()) {
            throw std::runtime_error("Unable to open odom file.");
        }
        odom_file << "time,field.header.seq,field.header.stamp,"
                  << "field.pose.pose.position.x,field.pose.pose.position.y,field.pose.pose.position.z,"
                  << "field.pose.pose.orientation.x,field.pose.pose.orientation.y,field.pose.pose.orientation.z,field.pose.pose.orientation.w,"
                  << "field.twist.twist.linear.x,field.twist.twist.linear.y,field.twist.twist.linear.z,"
                  << "field.twist.twist.angular.x,field.twist.twist.angular.y,field.twist.twist.angular.z,"
                  << "gbx,gby,gbz,abx,aby,abz\n";	
	}
	~ESKF() {

		 if (odom_file.is_open()) {
            odom_file.close();
        }
	}

	
	// EKF Update
	bool setup(double _T, double _calibTime,
        double _gyr_dev, double _gyr_rw_dev,
        double _acc_dev, double _acc_rw_dev,
        double x_init = 0.0, double y_init = 0.0, double z_init = 0.0,
        double roll_init = 0.0, double pitch_init = 0.0, double yaw_init = 0.0)
	{
		acc_dev = _acc_dev;
		gyr_dev = _gyr_dev;
		gyr_rw_dev = _gyr_rw_dev;
		acc_rw_dev = _acc_rw_dev;
		gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
		
		T = _T;
		T2 = _T * _T;
		calibTime = _calibTime;

		// Inicialización del estado nominal
		p = Eigen::Vector3d(x_init, y_init, z_init);
		v.setZero();
		
		Eigen::AngleAxisd rollAngle(roll_init, Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd pitchAngle(pitch_init, Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd yawAngle(yaw_init, Eigen::Vector3d::UnitZ());
		q = yawAngle * pitchAngle * rollAngle;
		q.normalize();

		ba.setZero();
		bg.setZero();

		// Configuración del buffer de calibración
		if(calibTime == 0.0) {
			calibSize = 1;
		} else {
			calibSize = (int)(_calibTime / _T);
			if (calibSize < 1) calibSize = 1;
		}
		
		calibData.resize(calibSize);
		calibIndex = 0;

		// Inicialización de covarianza (Error state: 15x15)
		P.setZero();

		init = false;
		sec = 0;
		
		return true;
	}

	// Input: last sensor_msgs::Imu
	// Input: last sensor_msgs::Imu
	bool initialize(sensor_msgs::msg::Imu &msg, double init_abx = 0.0, double init_aby = 0.0, double init_abz = 0.0)
	{

		if (calibSize <= 1 || calibTime < 0.1) {
            
            // A. Bias a CERO (Asumimos sensor ideal)
            bg.setZero(); 
            ba = Eigen::Vector3d(init_abx, init_aby, init_abz);
            
            // B. Gravedad Teórica (o la magnitud calibrada 9.73 si la sabes fija)
            gravity = Eigen::Vector3d(0.0, 0.0, -9.81); 

            // C. IMPORTANTE: NO recalculamos Roll/Pitch.
            // Mantenemos la 'q' intacta tal cual vino de setup() (valores del YAML).
            
            RCLCPP_WARN(rclcpp::get_logger("ESKF"), 
                "MOVING START (CalibTime=0). Skipping IMU alignment. Using YAML Pose.");
            
            init = true;
            return true; // <--- SALIMOS AQUÍ, antes de calcular nada mal.
        }
		// 1. Llenar el buffer de calibración
		calibData[calibIndex++ % calibSize] = msg;
		
		if(calibIndex < calibSize)
			return false;
		
		// 2. Calcular Medias
		double gx_m = 0.0, gy_m = 0.0, gz_m = 0.0;
		double ax_m = 0.0, ay_m = 0.0, az_m = 0.0;
		int samples = calibData.size();

		for(const auto& data : calibData)
		{
			if (!std::isnan(data.angular_velocity.x)) gx_m += data.angular_velocity.x;
			if (!std::isnan(data.angular_velocity.y)) gy_m += data.angular_velocity.y;
			if (!std::isnan(data.angular_velocity.z)) gz_m += data.angular_velocity.z;

			if (!std::isnan(data.linear_acceleration.x)) ax_m += data.linear_acceleration.x;
			if (!std::isnan(data.linear_acceleration.y)) ay_m += data.linear_acceleration.y;
			if (!std::isnan(data.linear_acceleration.z)) az_m += data.linear_acceleration.z;
		}

		gx_m /= samples; gy_m /= samples; gz_m /= samples;
		ax_m /= samples; ay_m /= samples; az_m /= samples;

		// -----------------------------------------------------------------------
		// 3. INICIALIZACIÓN DEL ESTADO (CORREGIDA)
		// -----------------------------------------------------------------------

		// A. Calcular Roll y Pitch basados en la gravedad (Acelerómetro)
		// Asumiendo sistema estándar: X-Front, Y-Left, Z-Up
		// ax = -g * sin(pitch)
		// ay = g * cos(pitch) * sin(roll)
		// az = g * cos(pitch) * cos(roll)
		
		double roll  = std::atan2(ay_m, az_m);
		double pitch = std::atan2(-ax_m, std::sqrt(ay_m*ay_m + az_m*az_m));

		// B. RECUPERAR EL YAW INICIAL (CRÍTICO)
		// El 'q' actual contiene la rotación del setup (incluyendo m_init_yaw).
		// Extraemos el Yaw actual para no perderlo.
		double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
		double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
		double current_yaw = std::atan2(siny_cosp, cosy_cosp);

		// C. Reconstruir el Cuaternión: (Yaw Original) * (Pitch Nuevo) * (Roll Nuevo)
		Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd yawAngle(current_yaw, Eigen::Vector3d::UnitZ());

		q = yawAngle * pitchAngle * rollAngle;
		q.normalize();

		// D. Inicializar Bias y Velocidad (Igual que antes)
		bg = Eigen::Vector3d(gx_m, gy_m, gz_m);
		ba = Eigen::Vector3d(init_abx, init_aby, init_abz);
		v.setZero();

		// 4. Inicialización de Covarianza (P) - Igual que antes
		P.setZero();
		P.block<3,3>(0,0).diagonal().fill(1e-4);   // Pos
		P.block<3,3>(3,3).diagonal().fill(1e-4);   // Vel
		P.block<3,3>(6,6).diagonal().fill(1e-5);   // Ori
		P.block<3,3>(9,9).diagonal().fill(1e-3);   // Bias Acc
		P.block<3,3>(12,12).diagonal().fill(1e-4); // Bias Gyr

		if(calibTime == 0.0){
			RCLCPP_WARN(rclcpp::get_logger("ESKF"), "Calibration skipped. Bias 0.");
			bg.setZero(); 
		} else {
			RCLCPP_INFO(rclcpp::get_logger("ESKF"), "Calibration Done. Samples: %d", samples);
			RCLCPP_INFO_STREAM(rclcpp::get_logger("ESKF"), "Init Gyro Bias: " << bg.transpose());
			// Log para verificar que el Yaw se ha mantenido
			RCLCPP_INFO(rclcpp::get_logger("ESKF"), "Init Orientation (RPY): %.4f, %.4f, %.4f", roll, pitch, current_yaw);
		}

		init = true;    
		return true;
	}
	
	
	// EKF prediction stage based on gyro information
	// Input: raw gyro sensors rad/s
	bool predict(double gx, double gy, double gz,double ax, double ay, double az,double stamp, double dt)
	{
		
		// Check initialization 
    	if(!init) return false;
        
		// 0. Preparar datos y paso de tiempo
		T = dt;
		T2 = dt * dt;

		// Vectorizar inputs (Mediciones crudas)
		Eigen::Vector3d acc_meas(ax, ay, az);
		Eigen::Vector3d gyr_meas(gx, gy, gz);

		Eigen::Vector3d acc_unbiased = acc_meas - ba;
		Eigen::Vector3d gyr_unbiased = gyr_meas - bg;

        Eigen::Matrix3d R = q.toRotationMatrix();
    	Eigen::Vector3d acc_world = R * acc_unbiased + gravity; // Aceleración en marco mundo

		p = p + v * T + 0.5 * acc_world * T2;
    	v = v + acc_world * T;
		
		Eigen::Vector3d angle_vec = gyr_unbiased * T;
		double angle_norm = angle_vec.norm();
		
		if (angle_norm > 1e-8) { // Evitar división por cero si estamos muy quietos
			Eigen::AngleAxisd delta_rot(angle_norm, angle_vec.normalized());
			q = q * delta_rot; // Orden: q_global = q_old * q_delta (rotación local)
			q.normalize();     // Vital normalizar siempre para evitar drift numérico
		}

		Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();

		// Bloque Posición-Velocidad (dp / dv) -> I * dt
		F.block<3,3>(0,3) = Eigen::Matrix3d::Identity() * T;

		// Bloque Velocidad-Orientación (dv / dtheta) -> -R * [a]x * dt
		// Este es el término crítico que conecta inclinación con aceleración
		F.block<3,3>(3,6) = -R * skew(acc_unbiased) * T;

		// Bloque Velocidad-BiasAccel (dv / dba) -> -R * dt
		F.block<3,3>(3,9) = -R * T;

		// Bloque Orientación-BiasGyro (dtheta / dbg) -> -I * dt
		F.block<3,3>(6,12) = -Eigen::Matrix3d::Identity() * T;

		// B. Construir Matriz de Ruido Q (Discreta)
		// Q diagonal simplificada para el proceso
		Eigen::Matrix<double, 15, 15> Q = Eigen::Matrix<double, 15, 15>::Zero();
		
		// Ruido Posición/Velocidad (Integrado del acelerómetro)
		// V = acc_noise * dt
		double sigma_acc_dt = acc_dev * T;
		double sigma_acc_dt2 = sigma_acc_dt * T; // Ruido pos es de segundo orden (opcional)
		
		Q.block<3,3>(3,3).diagonal().fill(sigma_acc_dt * sigma_acc_dt); // Ruido Vel
		// Q.block<3,3>(0,0) ... Ruido posición explícito es muy pequeño, suele dejarse a 0 o muy bajo

		// Ruido Orientación (Integrado del Gyro)
		double sigma_gyr_dt = gyr_dev * T;
		Q.block<3,3>(6,6).diagonal().fill(sigma_gyr_dt * sigma_gyr_dt);

		// Ruido Random Walk de los Bias (Bias cambia muy lento)
		double var_ba_rw = acc_rw_dev * acc_rw_dev * T; // sigma^2 * dt
		double var_bg_rw = gyr_rw_dev * gyr_rw_dev * T; // sigma^2 * dt

		Q.block<3,3>(9,9).diagonal().fill(var_ba_rw);
		Q.block<3,3>(12,12).diagonal().fill(var_bg_rw);

		// C. Actualizar P
		// P_new = F * P_old * F_transposed + Q
		// Nota: Si quieres optimizar velocidad, puedes multiplicar por bloques manualmente,
		// pero con Eigen esto suele ser suficientemente rápido (< 10us).
		P = F * P * F.transpose() + Q;
		
		// -----------------------------------------------------------------------
		// 3. LOGGING (Opcional, para debug)
		// -----------------------------------------------------------------------
		// Guardamos variables para visualización (igual que tenías antes)
		gxf = gx; gyf = gy; gzf = gz; // Guardar crudos para debug
		
		// Escribir en CSV (Asegúrate de actualizar qué escribes aquí porque rx, ry ya no existen)
		// Puedes extraer Roll/Pitch/Yaw de q para loguear:
		
		odom_file << std::fixed << std::setprecision(0)
				<< stamp * 1e9 << "," << sec << "," << stamp * 1e9 << ","  
				<< std::fixed << std::setprecision(9)
				<< p.x() << "," << p.y() << "," << p.z() << ","
				<< q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "," // Euler estimado
				<< v.x() << "," << v.y() << "," << v.z() << ","
				<< bg.x() << "," << bg.y() << "," << bg.z() << "," // Bias Gyro
				<< ba.x() << "," << ba.y() << "," << ba.z() << "\n"; // Bias Accel
				
		sec++;
		return true; 
	}


	// Update SOLO con Pose (Posición + Cuaternión)
    // Eficiente y estable: Deja que el filtro estime la velocidad internamente.
    bool update_pose(const Eigen::Vector3d& meas_p, 
                     const Eigen::Quaterniond& meas_q, 
                     double var_pos, double var_ori, 
                     double stamp)
    {
        if(!init) return false;

        // 1. CALCULAR RESIDUALES (Innovación)
        // ------------------------------------
        // A. Posición
        Eigen::Vector3d y_p = meas_p - p;

        // B. Orientación (Diferencia de Cuaterniones)
        Eigen::Quaterniond q_err = q.conjugate() * meas_q; 
        q_err.normalize();
        double sign_check = (q_err.w() >= 0) ? 1.0 : -1.0;
        Eigen::Vector3d y_theta = sign_check * 2.0 * q_err.vec(); 

        // Vector Residual Total (Ahora es 6x1 en vez de 9x1)
        Eigen::VectorXd y(6);
        y << y_p, y_theta;

        // 2. MATRICES DEL KALMAN (H, R)
        // -----------------------------
        
        // Matriz H (6x15): Mapea Estado (15) a Medición (6)
        // Observamos: Posición (indices 0-2) y Orientación (indices 6-8)
        // Saltamos la Velocidad (indices 3-5)
        Eigen::Matrix<double, 6, 15> H;
        H.setZero();
        H.block<3,3>(0,0).setIdentity(); // Medimos Posición
        // H.block<3,3>(?,3) -> Velocidad NO la medimos
        H.block<3,3>(3,6).setIdentity(); // Medimos Orientación (Ojo: indice 3 en H corresponde a 6 en Estado)

        // Matriz R (6x6): Ruido de la medición
        Eigen::Matrix<double, 6, 6> R_cov;
        R_cov.setZero();
        R_cov.block<3,3>(0,0).diagonal().fill(var_pos);
        R_cov.block<3,3>(3,3).diagonal().fill(var_ori);

        // 3. PASO DE CORRECCIÓN
        // ---------------------
        
        // S = H * P * Ht + R
        Eigen::Matrix<double, 6, 6> S = H * P * H.transpose() + R_cov;

        // K = P * Ht * S_inv
        Eigen::Matrix<double, 15, 6> K = P * H.transpose() * S.inverse();

        // Delta X = K * y (Vector de error de 15x1)
        Eigen::VectorXd delta_x = K * y;

        // Update P
        P = (Eigen::Matrix<double, 15, 15>::Identity() - K * H) * P;

        // 4. INYECCIÓN DEL ERROR (Igual que siempre)
        // ----------------------
        p  += delta_x.segment<3>(0);
        v  += delta_x.segment<3>(3); // <-- IMPORTANTE: ¡Aquí SÍ se corrige la velocidad!
                                     // El filtro deduce "si estaba más lejos de lo que pensaba, es que iba más rápido".
        ba += delta_x.segment<3>(9);
        bg += delta_x.segment<3>(12);

        Eigen::Vector3d delta_theta = delta_x.segment<3>(6);
        Eigen::Quaterniond dq;
        double theta_norm = delta_theta.norm();
        if(theta_norm > 1e-8) {
            dq = Eigen::AngleAxisd(theta_norm, delta_theta.normalized());
        } else {
            dq = Eigen::Quaterniond::Identity();
        }

        q = q * dq; 
        q.normalize(); 

        // 5. Logging (Igual)
        odom_file << std::fixed << std::setprecision(0)
                << stamp * 1e9 << "," << sec << "," << stamp * 1e9 << ","  
                << std::fixed << std::setprecision(9)
                << p.x() << "," << p.y() << "," << p.z() << ","
                << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "," 
                << v.x() << "," << v.y() << "," << v.z() << ","
                << gxf << "," << gyf << "," << gzf << "," 
                << bg.x() << "," << bg.y() << "," << bg.z() << ","
                << ba.x() << "," << ba.y() << "," << ba.z() << "\n";
        sec++;

        return true;
    }
	bool getEuler(double &roll, double &pitch, double &yaw)
    {
        // Convierte el cuaternión interno a matriz de rotación y extrae Euler XYZ
        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        
        roll  = euler[0];
        pitch = euler[1];
        yaw   = euler[2];
        
        return true;
    }

    // 2. Obtener Cuaternión (x, y, z, w)
    bool getQuat(double &qx, double &qy, double &qz, double &qw)
    {
        qx = q.x();
        qy = q.y();
        qz = q.z();
        qw = q.w();
        
        return true;
    }

	double get_sec(){
		return sec;
	}

	// Obtener Velocidades Globales
    bool getVelocities(double &vx_, double &vy_, double &vz_)
    {
        vx_ = v.x();
        vy_ = v.y();
        vz_ = v.z();
        return true;
    }

    // Obtener Posición
    bool getposition(double &x_, double &y_, double &z_)
    {
        x_ = p.x();
        y_ = p.y();
        z_ = p.z();
        return true;
    }

    // Obtener Bias del Giroscopio
    bool getBIAS(double &gbx_, double &gby_, double &gbz_)
    {
        gbx_ = bg.x();
        gby_ = bg.y();
        gbz_ = bg.z();
        return true;
    }
	
	bool isInit(void)
	{
		return init;
	}


	
protected:

	
	// --- ESKF State ---
    Eigen::Vector3d p;    // Posición Global
    Eigen::Vector3d v;    // Velocidad Global
    Eigen::Quaterniond q; // Orientación (Body -> Global)
    Eigen::Vector3d ba;   // Bias Acelerómetro
    Eigen::Vector3d bg;   // Bias Giroscopio

    // --- ESKF Covariance ---
    // Orden: [delta_p, delta_v, delta_theta, delta_ba, delta_bg]
    Eigen::Matrix<double, 15, 15> P;

    // --- Parámetros y Auxiliares ---
    double acc_dev, gyr_dev;
    double acc_rw_dev, gyr_rw_dev;
    double T, T2;
    double sec;
    bool init;

	Eigen::Vector3d gravity; 
    double gxf, gyf, gzf;    // Añadidos para logging


    // --- Calibración y Logs ---
    double calibTime;
    int calibIndex, calibSize;
    std::vector<sensor_msgs::msg::Imu> calibData;
    std::ofstream odom_file;


	Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
		Eigen::Matrix3d m;
		m << 0.0, -v(2), v(1),
			v(2), 0.0, -v(0),
			-v(1), v(0), 0.0;
		return m;
	}
	double Floor_absolute( double value )
	{
	  if (value < 0.0)
		return ceil( value );
	  else
		return floor( value );
	}

	//! Convert angles into interval [-PI,0,PI]
	double Pi2PiRange(double cont_angle)
	{
		double bound_angle = 0.0;
		if(fabs(cont_angle)<=M_PI)
			bound_angle= cont_angle;
		else
		{
			if(cont_angle > M_PI)
				bound_angle = (cont_angle-2*M_PI) - 2*M_PI*Floor_absolute((cont_angle-M_PI)/(2*M_PI));
			
			if(cont_angle < - M_PI)
				bound_angle = (cont_angle+2*M_PI) - 2*M_PI*Floor_absolute((cont_angle+M_PI)/(2*M_PI));
		}
		
		return bound_angle;
	}

		

};

#endif