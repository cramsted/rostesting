/**
 * @brief Conn class interface
 * @file conn_interface.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup conn
 * @{
 */
/*
 * Copyright 2013 Vladimir Ermakov.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <set>
#include <conn_interface.h>
#include <utils.h>

#include <conn_msgbuffer.h>
#include <conn_serial.h>

namespace conn {

std::set<int> ConnInterface::allocated_channels;
std::recursive_mutex ConnInterface::channel_mutex;


ConnInterface::ConnInterface()
{
	channel = new_channel();
    //ROS_ASSERT_MSG(channel >= 0, "channel allocation failure");
}

int ConnInterface::new_channel() {
	std::lock_guard<std::recursive_mutex> lock(channel_mutex);
	int chan = 0;

    for (chan = 0; chan < COMM_NUM_BUFFERS; chan++) {
		if (allocated_channels.count(chan) == 0) {
            //ROS_DEBUG_NAMED("conn", "Allocate new channel: %d", chan);
			allocated_channels.insert(chan);
			return chan;
		}
	}

    //ROS_ERROR_NAMED("conn", "channel overrun");
	return -1;
}

void ConnInterface::delete_channel(int chan) {
	std::lock_guard<std::recursive_mutex> lock(channel_mutex);
//	ROS_DEBUG_NAMED("conn", "Freeing channel: %d", chan);
	allocated_channels.erase(allocated_channels.find(chan));
}

int ConnInterface::channes_available() {
	std::lock_guard<std::recursive_mutex> lock(channel_mutex);
    return COMM_NUM_BUFFERS - allocated_channels.size();
}

MsgBuffer *ConnInterface::new_msgbuffer(const uint8_t *bytes, ssize_t nbytes)
{
        return new MsgBuffer(bytes, nbytes);
}

/**
 * Parse host:port pairs
 */
static void url_parse_host(std::string host,
		std::string &host_out, int &port_out,
		const std::string def_host, const int def_port)
{
	std::string port;

	auto sep_it = std::find(host.begin(), host.end(), ':');
	if (sep_it == host.end()) {
		// host
		if (!host.empty()) {
			host_out = host;
			port_out = def_port;
		}
		else {
			host_out = def_host;
			port_out = def_port;
		}
		return;
	}

	if (sep_it == host.begin()) {
		// :port
		host_out = def_host;
	}
	else {
		// host:port
		host_out.assign(host.begin(), sep_it);
	}

	port.assign(sep_it + 1, host.end());
	port_out = std::stoi(port);
}

/**
 * Parse ?ids=sid,cid
 */
static void url_parse_query(std::string query)
{
	const std::string ids_end("ids=");
	std::string sys, comp;

	if (query.empty())
		return;

	auto ids_it = std::search(query.begin(), query.end(),
			ids_end.begin(), ids_end.end());
	if (ids_it == query.end()) {
//		ROS_WARN_NAMED("conn", "URL: unknown query arguments");
		return;
	}

	std::advance(ids_it, ids_end.length());
	auto comma_it = std::find(ids_it, query.end(), ',');
	if (comma_it == query.end()) {
//		ROS_ERROR_NAMED("conn", "URL: no comma in ids= query");
		return;
	}

	sys.assign(ids_it, comma_it);
	comp.assign(comma_it + 1, query.end());
}

static ConnInterface::Ptr url_parse_serial(
        std::string path, int baud, std::string query)
{
	std::string file_path;
	int baudrate;

	// /dev/ttyACM0:57600
    url_parse_host(path, file_path, baudrate, "/dev/ttyACM0", baud);
    url_parse_query(query);

    return boost::make_shared<ConnSerial>(file_path, baudrate);
}

ConnInterface::Ptr ConnInterface::open_url(std::string url, int baudrate)
{

	/* Based on code found here:
	 * http://stackoverflow.com/questions/2616011/easy-way-to-parse-a-url-in-c-cross-platform
	 */

	const std::string proto_end("://");
	std::string proto;
	std::string host;
	std::string path;
	std::string query;

	auto proto_it = std::search(
			url.begin(), url.end(),
			proto_end.begin(), proto_end.end());
	if (proto_it == url.end()) {
		// looks like file path
//		ROS_DEBUG_NAMED("conn", "URL: %s: looks like file path", url.c_str());
        return url_parse_serial(url, baudrate, "");
	}

	// copy protocol
	proto.reserve(std::distance(url.begin(), proto_it));
	std::transform(url.begin(), proto_it,
			std::back_inserter(proto),
			std::ref(tolower));

	// copy host
	std::advance(proto_it, proto_end.length());
	auto path_it = std::find(proto_it, url.end(), '/');
	std::transform(proto_it, path_it,
			std::back_inserter(host),
			std::ref(tolower));

	// copy path, and query if exists
	auto query_it = std::find(path_it, url.end(), '?');
	path.assign(path_it, query_it);
	if (query_it != url.end())
		++query_it;
	query.assign(query_it, url.end());

//	ROS_DEBUG_NAMED("conn", "URL: %s: proto: %s, host: %s, path: %s, query: %s",
//			url.c_str(), proto.c_str(), host.c_str(), path.c_str(), query.c_str());


    if (proto == "serial")
        return url_parse_serial(path, baudrate, query);
	else
		throw DeviceError("url", "Unknown URL type");
}

}; // namespace conn
