/*
 * helpers.hpp
 *
 *  Created on: Jan 6, 2014
 *      Author: skybotix
 */

#ifndef HELPERS_HPP_
#define HELPERS_HPP_

#include <iostream>

//AtomicWriter usage:
// {
//   AtomicWriter w;
//   w << "vals: ";
//   for(int i=0; i<10; i++)
//	  w << i << " ";
// } //destructor will write...

class AtomicWriter {
	std::ostringstream st;
public:
	template<typename T>
	AtomicWriter& operator<<(T const& t) {
		st << t;
		return *this;
	}
	AtomicWriter() {}

	AtomicWriter(std::string str) {
		st << str;
	}

	~AtomicWriter() {
		flush();
	}

	void flush(void) {
		std::string s = st.str();
		std::cout << s;
	}
};

#endif /* HELPERS_HPP_ */
