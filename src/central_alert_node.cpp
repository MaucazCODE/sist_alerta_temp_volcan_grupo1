#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sist_alertatemp_volcan/msg/custom_msg.hpp"
#include "sist_alertatemp_volcan/srv/check_sensor_status.hpp"

struct SensorInfo
{
  std::string estado;
  float densidad;
  rclcpp::Time ultima_actualizacion;
};

class CentralAlertNode : public rclcpp::Node
{
public:
  using CustomMsg = sist_alertatemp_volcan::msg::CustomMsg;
  using CheckSensorStatus = sist_alertatemp_volcan::srv::CheckSensorStatus;

  CentralAlertNode() : Node("central_alert_node")
  {
    // 1. Declarar parámetros (se sobreescriben si se carga el archivo YAML)
    this->declare_parameter<double>("umbral_advertencia", 5.0);
    this->declare_parameter<double>("umbral_critico", 15.0);
    this->declare_parameter<double>("timeout_sensor_seg", 5.0);

    umbral_advertencia_ = this->get_parameter("umbral_advertencia").as_double();
    umbral_critico_ = this->get_parameter("umbral_critico").as_double();
    timeout_sensor_seg_ = this->get_parameter("timeout_sensor_seg").as_double();

    // 2. Callback para parámetros dinámicos en tiempo de ejecución
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&CentralAlertNode::on_parameter_change, this, std::placeholders::_1));

    // 3. Suscriptor al tópico de sensores con QoS Reliable + Transient Local
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    subscription_ = this->create_subscription<CustomMsg>(
      "custom_topic",
      qos_profile,
      std::bind(&CentralAlertNode::custom_msg_callback, this, std::placeholders::_1));

    // 4. Servicio síncrono para consultar el estado operativo de nodos remotos
    status_service_ = this->create_service<CheckSensorStatus>(
      "check_sensor_status",
      std::bind(
        &CentralAlertNode::handle_status_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      this->get_logger(),
      "Nodo centralizador iniciado | Umbral advertencia: %.2f | Umbral critico: %.2f | Timeout: %.1f s",
      umbral_advertencia_,
      umbral_critico_,
      timeout_sensor_seg_);
  }

private:
  double umbral_advertencia_;
  double umbral_critico_;
  double timeout_sensor_seg_;

  std::unordered_map<int32_t, SensorInfo> sensores_;

  rclcpp::Subscription<CustomMsg>::SharedPtr subscription_;
  rclcpp::Service<CheckSensorStatus>::SharedPtr status_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string evaluar_nivel_alerta(float densidad) const
  {
    if (densidad >= static_cast<float>(umbral_critico_)) {
      return "CRITICA";
    }
    if (densidad >= static_cast<float>(umbral_advertencia_)) {
      return "ADVERTENCIA";
    }
    return "NORMAL";
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parametros actualizados correctamente.";

    double nuevo_adv = umbral_advertencia_;
    double nuevo_crit = umbral_critico_;
    double nuevo_timeout = timeout_sensor_seg_;

    for (const auto & param : parameters) {
      if (param.get_name() == "umbral_advertencia") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result.successful = false;
          result.reason = "umbral_advertencia debe ser tipo double (ej: 5.0).";
          return result;
        }
        nuevo_adv = param.as_double();
      } else if (param.get_name() == "umbral_critico") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result.successful = false;
          result.reason = "umbral_critico debe ser tipo double (ej: 15.0).";
          return result;
        }
        nuevo_crit = param.as_double();
      } else if (param.get_name() == "timeout_sensor_seg") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result.successful = false;
          result.reason = "timeout_sensor_seg debe ser tipo double (ej: 5.0).";
          return result;
        }
        nuevo_timeout = param.as_double();
      }
    }

    if (nuevo_adv < 0.0 || nuevo_crit <= nuevo_adv || nuevo_timeout <= 0.0) {
      result.successful = false;
      result.reason = "Valores invalidos: requiere 0 <= umbral_advertencia < umbral_critico y timeout > 0.";
      RCLCPP_WARN(this->get_logger(), "Cambio de parametro rechazado: %s", result.reason.c_str());
      return result;
    }

    umbral_advertencia_ = nuevo_adv;
    umbral_critico_ = nuevo_crit;
    timeout_sensor_seg_ = nuevo_timeout;

    RCLCPP_INFO(
      this->get_logger(),
      "Parametros actualizados en caliente -> advertencia=%.2f, critico=%.2f, timeout=%.1f s",
      umbral_advertencia_,
      umbral_critico_,
      timeout_sensor_seg_);

    return result;
  }

  void custom_msg_callback(const CustomMsg::SharedPtr msg)
  {
    sensores_[msg->identificador] = SensorInfo{
      msg->estado,
      msg->densidad,
      this->now()
    };

    const std::string nivel = evaluar_nivel_alerta(msg->densidad);

    if (nivel == "CRITICA") {
      RCLCPP_ERROR(
        this->get_logger(),
        "[ALERTA CRITICA] Sensor #%d | densidad=%.2f kg/m3 (umbral=%.2f) | estado='%s'",
        msg->identificador,
        msg->densidad,
        umbral_critico_,
        msg->estado.c_str());
    } else if (nivel == "ADVERTENCIA") {
      RCLCPP_WARN(
        this->get_logger(),
        "[ADVERTENCIA] Sensor #%d | densidad=%.2f kg/m3 (umbral=%.2f) | estado='%s'",
        msg->identificador,
        msg->densidad,
        umbral_advertencia_,
        msg->estado.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "[NORMAL] Sensor #%d | densidad=%.2f kg/m3 | estado='%s'",
        msg->identificador,
        msg->densidad,
        msg->estado.c_str());
    }
  }

  void handle_status_request(
    const std::shared_ptr<CheckSensorStatus::Request> request,
    std::shared_ptr<CheckSensorStatus::Response> response)
  {
    const auto it = sensores_.find(request->sensor_id);
    if (it == sensores_.end()) {
      response->operativo = false;
      response->estado_sensor = "DESCONOCIDO";
      response->nivel_alerta = "SIN_DATOS";
      response->ultima_densidad = 0.0f;
      response->mensaje = "No se han recibido lecturas del sensor #" +
        std::to_string(request->sensor_id) + ".";
      RCLCPP_WARN(this->get_logger(), "Consulta servicio: %s", response->mensaje.c_str());
      return;
    }

    const SensorInfo & info = it->second;
    const double segundos_sin_datos = (this->now() - info.ultima_actualizacion).seconds();
    const bool en_linea = segundos_sin_datos <= timeout_sensor_seg_;

    response->operativo = en_linea;
    response->estado_sensor = info.estado;
    response->ultima_densidad = info.densidad;
    response->nivel_alerta = evaluar_nivel_alerta(info.densidad);

    if (en_linea) {
      response->mensaje = "Sensor #" + std::to_string(request->sensor_id) +
        " operativo (ultima lectura hace " +
        std::to_string(static_cast<int>(std::round(segundos_sin_datos))) + " s).";
    } else {
      response->mensaje = "Sensor #" + std::to_string(request->sensor_id) +
        " FUERA DE LINEA (sin señal hace " +
        std::to_string(static_cast<int>(std::round(segundos_sin_datos))) + " s).";
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Servicio consultado para sensor #%d -> operativo=%s, nivel=%s",
      request->sensor_id,
      response->operativo ? "true" : "false",
      response->nivel_alerta.c_str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CentralAlertNode>());
  rclcpp::shutdown();
  return 0;
}