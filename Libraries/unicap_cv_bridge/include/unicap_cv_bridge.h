//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        unicap cv bridge
// File:           unicap_cv_bridge.hpp
// Description:    symbolizes a unicap camera. frames can be grabbed and put into matrices
// Author:         Lukas Vermond & Kasper van Nieuwland
// Notes:          
//
// License:        GNU GPL v3
//
// This file is part of unicap cv bridge.
//
// unicap cv bridge is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// unicap cv bridge is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with unicap cv bridge.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************


#pragma once

#define private reserved
#include <unicap.h>
#include <unicap_status.h>
#undef private
#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include <stdexcept>
#include <string>
#include <vector>
#include <iostream>

namespace unicap_cv_bridge
{
	/**
	 * @brief obtain a list of device names
	 * @param devices output parameter. will contain device names
	 **/
	void list_devices(std::vector<std::string>& devices);
	
	/**
	 * @brief obtain a list of formats
	 * @param dev the device number
	 * @param formats output parameter. will contain format desciptions
	 **/
	void list_formats(int dev, std::vector<std::string>& formats);

	/**
	 * @brief obtain a list of properties
	 * @param dev the device number
	 * @param properties output parameter. will contain properties
	 **/
	void list_properties(int dev, std::vector<std::string>& properties);
	/**
	* @brief print a list of all connected devices with it's formats
	* @param out the output stream. Defaults to standard out
	**/
	void print_devices_inventory(std::ostream& out = std::cout);

	/**
	 * @brief unicap exception
	 **/
	class unicap_cv_exception : public std::runtime_error
	{
		public:
			unicap_cv_exception(const std::string& msg) : std::runtime_error(msg) {}
			virtual ~unicap_cv_exception(void) throw() {}
	};

	/**
	 * @brief handle to unicap camera
	 **/
	class unicap_cv_camera
	{
		private:
			typedef enum  _frame_cap_state
			{
				FCSTATE_DONT_COPY,
				FCSTATE_COPY,
				FCSTATE_SUCCESS,
				FCSTATE_FAILURE
			} frame_cap_state;
		
			bool tripmode;

			unicap_handle_t handle;
			unicap_device_t device;
			unicap_format_t format;
        
			boost::mutex mut;
			boost::condition_variable cond;
			frame_cap_state state;
			cv::Mat* mat;
        
		public:
			/**
			 * @brief constructs a unicap camera
			 * @param dev device number
			 * @param format format number
			 **/
			unicap_cv_camera(int dev, int format);

			~unicap_cv_camera(void);

			void set_trip_mode(bool tripmode);

			/**
			 * @brief get white balance settings
			 * @param blue out parameter
			 * @param red out parameter
			 **/
			void get_white_balance(double& blue, double& red);

			/**
			 * @brief set white balance settings
			 * @param blue blue
			 * @param red red
			 **/
			void set_white_balance(double blue, double red);

			/**
			 * @brief set exposure
			 * @param exposure exposure
			 **/
			void set_exposure(double exposure);

			/**
			 * @brief set auto white balance
			 * @param automatic true for on, false otherwise
			 **/
			void set_auto_white_balance(bool automatic);

			/**
			 * @brief get outputed image width
			 * @return image width
			 **/
			int get_img_width(void);

			/**
			 * @brief get outputed image height
			 * @return image height
			 **/
			int get_img_height(void);

			/**
			 * @brief get outputed image format
			 * @return image format
			 **/
			int get_img_format(void);

			/**
			 * @brief callback function which gets called when a new frame is captured
			 * @note should not be called by user
			 * @param buffer buffer which holds the frame
			 **/        
			void new_frame_cb(unicap_data_buffer_t* buffer);

			/**
			 * @brief get a frame
			 * 
			 * this function blocks until a new frame is captured
			 * @param mat the frame is copied into this matrix
			 **/        
			void get_frame(cv::Mat* mat);
	};
}
