/* Author: 
 * Subscribers:
 *    - /topic: description
 * Publishers:
 *    - /topic: description
 * Services:
 *    - /name (type): description
 *
 * - Summary
 * 
 * TODO
 * - Add todos
 * */

#include "lx_operation/operations_handler.hpp"

OperationsHandler::OperationsHandler(): Node("operations_handler_node"){

}

std::queue<std::shared_ptr<Task>, std::list<std::shared_ptr<Task>>> OperationsHandler::planner(){

}