/**
 * @file Utilities.h
 * @brief Utility functions to be used in the main project.
 * @date Created: 2012-10-22
 *
 * @author Koen Braham
 * @author Daan Veltman
 *
 * @section LICENSE
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/
#ifndef UTILITIES_H_
#define UTILITIES_H_

namespace Utilities {

	/**
	 * Converts a string to integer
	 * @param i Reference to integer to be assigned
	 * @param s String that contains the value
	 * @param base A value between 2 and 36 inclusive, which determines the base of the value in the string. Special value is 0, which takes the value as base 10 unless a prefix of 0x (hexadecimal) or 0 (octal).
	 *
	 * @return error code
	 * 	0 is normal
	 *	1 is overflow
	 *	2 is underflow
	 *	3 is inconvertible
	 **/
	int str2int(int &i, char const *s, int base = 0) {
		char *end;
		long l;
		errno = 0;
		l = strtol(s, &end, base);
		if ((errno == ERANGE && l == LONG_MAX) || l > INT_MAX) {
			return 1;
		}
		if ((errno == ERANGE && l == LONG_MIN) || l < INT_MIN) {
			return 2;
		}
		if (*s == '\0' || *end != '\0') {
			return 3;
		}
		i = l;

		return 0;
	}

} /* namespace Utilities */
#endif /* UTILITIES_H_ */