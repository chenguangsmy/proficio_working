#ifndef HAPPY_HELPERS_HELPING
template <size_t DOF>
void * responderWrapper(void*);

template <size_t DOF>
void * moveRobot(void*);

template<size_t DOF>
void recoverBurt(barrett::ProductManager&, barrett::systems::Wam<DOF>&)
#endif
