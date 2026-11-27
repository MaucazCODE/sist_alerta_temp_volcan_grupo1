#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sist_alertatemp_volcan/srv/check_sensor_status.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int32_t sensor_id = 1;
  if (argc > 1) {
    try {
      sensor_id = std::stoi(argv[1]);
    } catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("status_service_client"), "ID de sensor invalido.");
      rclcpp::shutdown();
      return 1;
    }
  }

  auto node = rclcpp::Node::make_shared("status_service_client");
  auto client = node->create_client<sist_alertatemp_volcan::srv::CheckSensorStatus>(
    "check_sensor_status");

  while (!client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrumpido mientras esperaba el servicio.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Esperando al servicio /check_sensor_status...");
  }

  auto request = std::make_shared<sist_alertatemp_volcan::srv::CheckSensorStatus::Request>();
  request->sensor_id = sensor_id;

  auto future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto res = future.get();
    RCLCPP_INFO(
      node->get_logger(),
      "Respuesta Sensor #%d -> operativo=%s | estado='%s' | nivel=%s | densidad=%.2f kg/m3 | msg='%s'",
      sensor_id,
      res->operativo ? "SI" : "NO",
      res->estado_sensor.c_str(),
      res->nivel_alerta.c_str(),
      res->ultima_densidad,
      res->mensaje.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Error al llamar al servicio check_sensor_status.");
  }

  rclcpp::shutdown();
  return 0;
}