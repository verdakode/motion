async def get_actuators_state(
    self,
    actuator_ids: list[int] | None = None,
) -> actuator_pb2.GetActuatorsStateResponse:
    request = actuator_pb2.GetActuatorsStateRequest(actuator_ids=actuator_ids or [])
    return await self.stub.GetActuatorsState(request) 