/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file protocol_splitter.cpp
 * NuttX Driver to multiplex mavlink and RTPS on a single serial port.
 * Makes sure the two protocols can be read & written simultanously by 2 processes.
 * It will create two devices:
 *    /dev/mavlink
 *    /dev/rtps
 */

#include <lib/cdev/CDev.hpp>
#include <px4_platform_common/sem.hpp>
#include <px4_platform_common/log.h>

#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdint>
#include <string.h>

class Mavlink2Dev;
class RtpsDev;
class ReadBuffer;

extern "C" __EXPORT int protocol_splitter_main(int argc, char *argv[]);

static uint8_t crc8table[] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
	0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
	0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
	0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
	0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
	0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
	0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
	0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
	0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
	0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
	0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3,
};

uint8_t crc8(const uint8_t *data, size_t len)
{
	assert(data);
	uint8_t crc = 0;

	while (len--) {
		crc = crc8table[crc ^ *data++];
	}

	return crc;
}

/*
struct Sp2Header {
	char magic;
	uint8_t type:2;
	uint16_t payload_len:14;
	uint8_t header_crc;
}
*/

const char Sp2HeaderMagic = 'S';
const int  Sp2HeaderSize  = 4;

struct StaticData {
	Mavlink2Dev *mavlink2;
	RtpsDev *rtps;
	sem_t r_lock;
	sem_t w_lock;
	char device_name[16];
	ReadBuffer *read_buffer;
};

namespace
{
static StaticData *objects = nullptr;
}

class ReadBuffer
{
public:
	int read(int fd);
	void copy(void *dest, size_t pos, size_t n);
	void remove(size_t pos, size_t n);

	uint8_t buffer[512] = {};
	size_t buf_size = 0;

	static const size_t BUFFER_THRESHOLD = sizeof(buffer) * 0.8;
};

int ReadBuffer::read(int fd)
{
	/* Discard whole buffer if it's filled beyond a threshold,
	 * This should prevent buffer being filled by garbage that
	 * no reader (MAVLink or RTPS) can understand.
	 *
	 * TODO: a better approach would be checking if both reader
	 * start understanding messages beyond a certain buffer size,
	 * meaning that everything before is garbage.
	 */
	if (buf_size > BUFFER_THRESHOLD) {
		buf_size = 0;
	}

	int r = ::read(fd, buffer + buf_size, sizeof(buffer) - buf_size);

	if (r < 0) {
		return r;
	}

	buf_size += r;

	return r;
}

void ReadBuffer::copy(void *dest, size_t pos, size_t n)
{
	ASSERT(pos < buf_size);
	ASSERT(pos + n <= buf_size);

	if (dest) {
		memmove(dest, buffer + pos, n);        // send desired data
	}
}

void ReadBuffer::remove(size_t pos, size_t n)
{
	ASSERT(pos < buf_size);
	ASSERT(pos + n <= buf_size);

	memmove(buffer + pos, buffer + (pos + n), sizeof(buffer) - pos - n);
	buf_size -= n;
}

class DevCommon : public cdev::CDev
{
public:
	DevCommon(const char *device_path);
	virtual ~DevCommon();

	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int	open(file *filp);
	virtual int	close(file *filp);

	enum Operation {Read, Write};

protected:

	enum MessageType {Mavlink = 0, Rtps};

	uint8_t _header[4] = {};

	virtual pollevent_t poll_state(struct file *filp);


	void lock(enum Operation op)
	{
		sem_t *this_lock = op == Read ? &objects->r_lock : &objects->w_lock;

		while (sem_wait(this_lock) != 0) {
			/* The only case that an error should occur here is if
			 * the wait was awakened by a signal.
			 */
			ASSERT(get_errno() == EINTR);
		}
	}

	void unlock(enum Operation op)
	{
		sem_t *this_lock = op == Read ? &objects->r_lock : &objects->w_lock;
		sem_post(this_lock);
	}

	int _fd = -1;

	uint16_t _packet_len;
	enum class ParserState : uint8_t {
		Idle = 0,
		GotLength
	};
	ParserState _parser_state = ParserState::Idle;

	bool _had_data = false; ///< whether poll() returned available data

private:
};

DevCommon::DevCommon(const char *device_path)
	: CDev(device_path)
{
}

DevCommon::~DevCommon()
{
	if (_fd >= 0) {
		::close(_fd);
	}
}

int DevCommon::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	//pretend we have enough space left to write, so mavlink will not drop data and throw off
	//our parsing state
	if (cmd == FIONSPACE) {
		*(int *)arg = 1024;
		return 0;
	}

	return ::ioctl(_fd, cmd, arg);
}

int DevCommon::open(file *filp)
{
	_fd = ::open(objects->device_name, O_RDWR | O_NOCTTY);
	CDev::open(filp);
	return _fd >= 0 ? 0 : -1;
}

int DevCommon::close(file *filp)
{
	//int ret = ::close(_fd); // FIXME: calling this results in a dead-lock, because DevCommon::close()
	// is called from within another close(), and NuttX seems to hold a semaphore at this point
	_fd = -1;
	CDev::close(filp);
	return 0;
}

pollevent_t DevCommon::poll_state(struct file *filp)
{
	pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	/* Here we should just check the poll state (which is called before an actual poll waiting).
	 * Instead we poll on the fd with some timeout, and then pretend that there is data.
	 * This will let the calling poll return immediately (there's still no busy loop since
	 * we do actually poll here).
	 * We do this because there is no simple way with the given interface to poll on
	 * the _fd in here or by overriding some other method.
	 */

	int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), 100);
	_had_data = ret > 0 && (fds[0].revents & POLLIN);

	return POLLIN;
}

class Mavlink2Dev : public DevCommon
{
public:
	Mavlink2Dev(ReadBuffer *_read_buffer);
	virtual ~Mavlink2Dev() {}

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t buflen);

protected:
	ReadBuffer *_read_buffer;
	size_t _remaining_partial = 0;
	size_t _partial_start = 0;
	uint8_t _partial_buffer[512] = {};
};

Mavlink2Dev::Mavlink2Dev(ReadBuffer *read_buffer)
	: DevCommon("/dev/mavlink")
	, _read_buffer{read_buffer}
{
	_header[0] = Sp2HeaderMagic;
	_header[1] = (MessageType::Mavlink & 0x3) << 6;
	_header[2] = 0;
	_header[3] = 0;
}

ssize_t Mavlink2Dev::read(struct file *filp, char *buffer, size_t buflen)
{
	int i, ret;
	uint16_t packet_len, payload_len;

	/* last reading was partial (i.e., buffer didn't fit whole message),
	 * so now we'll just send remaining bytes */
	if (_remaining_partial > 0) {
		size_t len = _remaining_partial;

		if (buflen < len) {
			len = buflen;
		}

		memmove(buffer, _partial_buffer + _partial_start, len);
		_partial_start += len;
		_remaining_partial -= len;

		if (_remaining_partial == 0) {
			_partial_start = 0;
		}

		return len;
	}

	if (!_had_data) {
		return 0;
	}

	lock(Read);
	ret = _read_buffer->read(_fd);

	if (ret < 0) {
		goto end;
	}

	ret = 0;

	if (_read_buffer->buf_size < Sp2HeaderSize) {
		goto end;
	}

	// Search for a mavlink packet on buffer to send it
	i = 0;

	while ((unsigned)i < (_read_buffer->buf_size - Sp2HeaderSize) &&
	       (_read_buffer->buffer[i] != 'S'
		|| (_read_buffer->buffer[i + 1] >> 6) != (uint8_t) MessageType::Mavlink
		|| (_read_buffer->buffer[i + 3] != crc8(&_read_buffer->buffer[i], 3)))
	      ) {
		i++;
	}

	// We need at least the first six bytes to get packet len
	if ((unsigned)i >= _read_buffer->buf_size - Sp2HeaderSize) {
		goto end;
	}

	payload_len = ((uint16_t)(_read_buffer->buffer[i + 1] & 0x3f) << 8) | _read_buffer->buffer[i + 2];
	packet_len = payload_len + Sp2HeaderSize;

	// packet is bigger than what we've read, better luck next time
	if ((unsigned)i + packet_len > _read_buffer->buf_size) {
		goto end;
	}

	/* if buffer doesn't fit message, send what's possible and copy remaining
	 * data into a temporary buffer on this class */
	if (payload_len > buflen) {
		_read_buffer->copy(buffer, i + Sp2HeaderSize, buflen);
		_read_buffer->copy(_partial_buffer, i + Sp2HeaderSize + buflen, payload_len - buflen);
		_read_buffer->remove(i, packet_len);
		_remaining_partial = payload_len - buflen;
		ret = buflen;
		goto end;
	}

	_read_buffer->copy(buffer, i + Sp2HeaderSize, payload_len);
	_read_buffer->remove(i, packet_len);

	ret = payload_len;

end:
	unlock(Read);
	return ret;
}

ssize_t Mavlink2Dev::write(struct file *filp, const char *buffer, size_t buflen)
{
	/*
	 * we need to look into the data to make sure the output is locked for the duration
	 * of a whole packet.
	 * assumptions:
	 * - packet header is written all at once (or at least it contains the payload length)
	 * - a single write call does not contain multiple (or parts of multiple) packets
	 */
	ssize_t ret = 0;

	switch (_parser_state) {
	case ParserState::Idle:
		ASSERT(buflen >= 3);

		if ((unsigned char)buffer[0] == 253) {
			uint8_t payload_len = buffer[1];
			uint8_t incompat_flags = buffer[2];
			_packet_len = payload_len + 12;

			if (incompat_flags & 0x1) { //signing
				_packet_len += 13;
			}

			_parser_state = ParserState::GotLength;
			lock(Write);

		} else if ((unsigned char)buffer[0] == 254) { // mavlink 1
			uint8_t payload_len = buffer[1];
			_packet_len = payload_len + 8;

			_parser_state = ParserState::GotLength;
			lock(Write);

		} else {
			PX4_ERR("parser error");
			return 0;
		}

	/* FALLTHROUGH */

	case ParserState::GotLength: {
			_packet_len -= buflen;
			int buf_free;
			::ioctl(_fd, FIONSPACE, (unsigned long)&buf_free);

			if (buf_free < (int)buflen) {
				//let write fail, to let mavlink know the buffer would overflow
				//(this is because in the ioctl we pretend there is always enough space)
				ret = -1;

			} else {
				_header[1] = (_header[1] & 0xC0) | (uint8_t)((buflen >> 8) & 0x3f);
				_header[2] = (uint8_t)(buflen & 0xff);
				_header[3] = crc8(_header, 3);
				::write(_fd, _header, Sp2HeaderSize);
				ret = ::write(_fd, buffer, buflen);
			}

			if (_packet_len == 0) {
				unlock(Write);
				_parser_state = ParserState::Idle;
			}
		}

		break;
	}

	return ret;
}

class RtpsDev : public DevCommon
{
public:
	RtpsDev(ReadBuffer *_read_buffer);
	virtual ~RtpsDev() {}

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t buflen);

protected:
	ReadBuffer *_read_buffer;

	static const uint8_t HEADER_SIZE = 9;
};

RtpsDev::RtpsDev(ReadBuffer *read_buffer)
	: DevCommon("/dev/rtps")
	, _read_buffer{read_buffer}
{
	_header[0] = Sp2HeaderMagic;
	_header[1] = (MessageType::Rtps & 0x3) << 6;
	_header[2] = 0;
	_header[3] = 0;
}

ssize_t RtpsDev::read(struct file *filp, char *buffer, size_t buflen)
{
	int i, ret;
	uint16_t packet_len, payload_len;

	if (!_had_data) {
		return 0;
	}

	lock(Read);
	ret = _read_buffer->read(_fd);

	if (ret < 0) {
		goto end;
	}

	ret = 0;

	if (_read_buffer->buf_size < Sp2HeaderSize) {
		goto end;
	}

	// Search for a rtps packet on buffer to send it
	i = 0;

	while ((unsigned)i < (_read_buffer->buf_size - Sp2HeaderSize) &&
	       (_read_buffer->buffer[i] != 'S'
		|| (_read_buffer->buffer[i + 1] >> 6) != (uint8_t) MessageType::Rtps
		|| (_read_buffer->buffer[i + 3] != crc8(&_read_buffer->buffer[i], 3)))
	      ) {
		i++;
	}

	// We need at least the first six bytes to get packet len
	if ((unsigned)i >= _read_buffer->buf_size - Sp2HeaderSize) {
		goto end;
	}

	payload_len = ((uint16_t)(_read_buffer->buffer[i + 1] & 0x3f) << 8) | _read_buffer->buffer[i + 2];
	packet_len = payload_len + Sp2HeaderSize;

	// packet is bigger than what we've read, better luck next time
	if ((unsigned)i + packet_len > _read_buffer->buf_size) {
		goto end;
	}

	// buffer should be big enough to hold a rtps packet
	if (packet_len > buflen) {
		ret = -EMSGSIZE;
		goto end;
	}

	_read_buffer->copy(buffer, i + Sp2HeaderSize, payload_len);
	_read_buffer->remove(i, packet_len);
	ret = payload_len;

end:
	unlock(Read);
	return ret;
}

ssize_t RtpsDev::write(struct file *filp, const char *buffer, size_t buflen)
{
	/*
	 * we need to look into the data to make sure the output is locked for the duration
	 * of a whole packet.
	 * assumptions:
	 * - packet header is written all at once (or at least it contains the payload length)
	 * - a single write call does not contain multiple (or parts of multiple) packets
	 */
	ssize_t ret = 0;
	uint16_t payload_len;

	switch (_parser_state) {
	case ParserState::Idle:
		ASSERT(buflen >= HEADER_SIZE);

		if (memcmp(buffer, ">>>", 3) != 0) {
			PX4_ERR("parser error");
			return 0;
		}

		payload_len = ((uint16_t)buffer[5] << 8) | buffer[6];
		_packet_len = payload_len + HEADER_SIZE;
		_parser_state = ParserState::GotLength;
		lock(Write);

	/* FALLTHROUGH */

	case ParserState::GotLength: {
			_packet_len -= buflen;
			int buf_free;
			::ioctl(_fd, FIONSPACE, (unsigned long)&buf_free);

			// TODO should I care about this for rtps?
			if ((unsigned)buf_free < buflen) {
				//let write fail, to let rtps know the buffer would overflow
				//(this is because in the ioctl we pretend there is always enough space)
				ret = -1;

			} else {
				_header[1] = (_header[1] & 0xC0) | (uint8_t)((buflen >> 8) & 0x3f);
				_header[2] = (uint8_t)(buflen & 0xff);
				_header[3] = crc8(_header, 3);
				::write(_fd, _header, Sp2HeaderSize);
				ret = ::write(_fd, buffer, buflen);
			}

			if (_packet_len == 0) {
				unlock(Write);
				_parser_state = ParserState::Idle;
			}
		}

		break;
	}

	return ret;
}

int protocol_splitter_main(int argc, char *argv[])
{
	if (argc < 2) {
		goto out;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		if (objects) {
			PX4_ERR("already running");
			return 1;
		}

		if (argc != 3) {
			goto out;
		}

		objects = new StaticData();

		if (!objects) {
			PX4_ERR("alloc failed");
			return -1;
		}

		strncpy(objects->device_name, argv[2], sizeof(objects->device_name));
		sem_init(&objects->r_lock, 1, 1);
		sem_init(&objects->w_lock, 1, 1);
		objects->read_buffer = new ReadBuffer();
		objects->mavlink2 = new Mavlink2Dev(objects->read_buffer);
		objects->rtps = new RtpsDev(objects->read_buffer);

		if (!objects->mavlink2 || !objects->rtps) {
			delete objects->mavlink2;
			delete objects->rtps;
			delete objects->read_buffer;
			sem_destroy(&objects->r_lock);
			sem_destroy(&objects->w_lock);
			delete objects;
			objects = nullptr;
			PX4_ERR("alloc failed");
			return -1;

		} else {
			objects->mavlink2->init();
			objects->rtps->init();
		}
	}

	if (!strcmp(argv[1], "stop")) {
		if (objects) {
			delete objects->mavlink2;
			delete objects->rtps;
			delete objects->read_buffer;
			sem_destroy(&objects->r_lock);
			sem_destroy(&objects->w_lock);
			delete objects;
			objects = nullptr;
		}
	}

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status")) {
		if (objects) {
			PX4_INFO("running");

		} else {
			PX4_INFO("not running");
		}
	}

	return 0;

out:
	PX4_ERR("unrecognized command, try 'start <device>', 'stop', 'status'");
	return 1;
}
