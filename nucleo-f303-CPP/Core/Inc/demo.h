/*
 * demo.h
 *
 *  Created on: Apr 21, 2020
 *      Author: martti
 */

#ifndef SRC_DEMO_H_
#define SRC_DEMO_H_

#include <stdint.h>
#include <list>
#include <vector>

namespace my_demo {

template <typename T> struct Q {
	T x;
	T y;
	T z;
};

class demo {
public:
	demo();
	virtual ~demo();
private:
	std::list<int32_t> lista;
};

} /* namespace my_demo */

#endif /* SRC_DEMO_H_ */
