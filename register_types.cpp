/* register_types.cpp */
#include "register_types.h"
//#include "class_db.h"

#include "smoke.h"

void register_smoke_types() {
	ClassDB::register_class<Smoke>();
}

void unregister_smoke_types() {
	//nothing to do here
}
