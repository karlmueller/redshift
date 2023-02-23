//
// asio.hpp
// ~~~~~~~~
//
// Copyright (c) 2003-2019 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef ASIO_HPP
#define ASIO_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#if defined(ESP_PLATFORM)
# include "esp_asio_config.h"
#endif // defined(ESP_PLATFORM)

#include "asio/associated_allocator.hpp"
#include "asio/associated_executor.hpp"
#include "asio/async_result.hpp"
#include "asio/awaitable.hpp"
#include "asio/basic_datagram_socket.hpp"
#include "asio/basic_deadline_timer.hpp"
#include "asio/basic_io_object.hpp"
#include "asio/basic_raw_socket.hpp"
#include "asio/basic_seq_packet_socket.hpp"
#include "asio/basic_serial_port.hpp"
#include "asio/basic_signal_set.hpp"
#include "asio/basic_socket.hpp"
#include "asio/basic_socket_acceptor.hpp"
#include "asio/basic_socket_iostream.hpp"
#include "asio/basic_socket_streambuf.hpp"
#include "asio/basic_stream_socket.hpp"
#include "asio/basic_streambuf.hpp"
#include "asio/basic_waitable_timer.hpp"
#include "asio/bind_executor.hpp"
#include "asio/buffer.hpp"
#include "asio/buffered_read_stream_fwd.hpp"
#include "asio/buffered_read_stream.hpp"
#include "asio/buffered_stream_fwd.hpp"
#include "asio/buffered_stream.hpp"
#include "asio/buffered_write_stream_fwd.hpp"
#include "asio/buffered_write_stream.hpp"
#include "asio/buffers_iterator.hpp"
#include "asio/co_spawn.hpp"
#include "asio/completion_condition.hpp"
#include "asio/compose.hpp"
#include "asio/connect.hpp"
#include "asio/coroutine.hpp"
#include "asio/deadline_timer.hpp"
#include "asio/defer.hpp"
#include "asio/detached.hpp"
#include "asio/dispatch.hpp"
#include "asio/error.hpp"
#include "asio/error_code.hpp"
#include "asio/execution_context.hpp"
#include "asio/executor.hpp"
#include "asio/executor_work_guard.hpp"
#include "asio/generic/basic_endpoint.hpp"
#include "asio/generic/datagram_protocol.hpp"
#include "asio/generic/raw_protocol.hpp"
#include "asio/generic/seq_packet_protocol.hpp"
#include "asio/generic/stream_protocol.hpp"
#include "asio/handler_alloc_hook.hpp"
#include "asio/handler_continuation_hook.hpp"
#include "asio/handler_invoke_hook.hpp"
#include "asio/high_resolution_timer.hpp"
#include "asio/io_context.hpp"
#include "asio/io_context_strand.hpp"
#include "asio/io_service.hpp"
#include "asio/io_service_strand.hpp"
#include "asio/ip/address.hpp"
#include "asio/ip/address_v4.hpp"
#include "asio/ip/address_v4_iterator.hpp"
#include "asio/ip/address_v4_range.hpp"
#include "asio/ip/address_v6.hpp"
#include "asio/ip/address_v6_iterator.hpp"
#include "asio/ip/address_v6_range.hpp"
#include "asio/ip/network_v4.hpp"
#include "asio/ip/network_v6.hpp"
#include "asio/ip/bad_address_cast.hpp"
#include "asio/ip/basic_endpoint.hpp"
#include "asio/ip/basic_resolver.hpp"
#include "asio/ip/basic_resolver_entry.hpp"
#include "asio/ip/basic_resolver_iterator.hpp"
#include "asio/ip/basic_resolver_query.hpp"
#include "asio/ip/host_name.hpp"
#include "asio/ip/icmp.hpp"
#include "asio/ip/multicast.hpp"
#include "asio/ip/resolver_base.hpp"
#include "asio/ip/resolver_query_base.hpp"
#include "asio/ip/tcp.hpp"
#include "asio/ip/udp.hpp"
#include "asio/ip/unicast.hpp"
#include "asio/ip/v6_only.hpp"
#include "asio/is_executor.hpp"
#include "asio/is_read_buffered.hpp"
#include "asio/is_write_buffered.hpp"
#include "asio/local/basic_endpoint.hpp"
#include "asio/local/connect_pair.hpp"
#include "asio/local/datagram_protocol.hpp"
#include "asio/local/stream_protocol.hpp"
#include "asio/packaged_task.hpp"
#include "asio/placeholders.hpp"
#include "asio/posix/basic_descriptor.hpp"
#include "asio/posix/basic_stream_descriptor.hpp"
#include "asio/posix/descriptor.hpp"
#include "asio/posix/descriptor_base.hpp"
#include "asio/posix/stream_descriptor.hpp"
#include "asio/post.hpp"
#include "asio/read.hpp"
#include "asio/read_at.hpp"
#include "asio/read_until.hpp"
#include "asio/redirect_error.hpp"
#include "asio/serial_port.hpp"
#include "asio/serial_port_base.hpp"
#include "asio/signal_set.hpp"
#include "asio/socket_base.hpp"
#include "asio/steady_timer.hpp"
#include "asio/strand.hpp"
#include "asio/streambuf.hpp"
#include "asio/system_context.hpp"
#include "asio/system_error.hpp"
#include "asio/system_executor.hpp"
#include "asio/system_timer.hpp"
#include "asio/this_coro.hpp"
#include "asio/thread.hpp"
#include "asio/thread_pool.hpp"
#include "asio/time_traits.hpp"
#include "asio/use_awaitable.hpp"
#include "asio/use_future.hpp"
#include "asio/uses_executor.hpp"
#include "asio/version.hpp"
#include "asio/wait_traits.hpp"
#include "asio/windows/basic_object_handle.hpp"
#include "asio/windows/basic_overlapped_handle.hpp"
#include "asio/windows/basic_random_access_handle.hpp"
#include "asio/windows/basic_stream_handle.hpp"
#include "asio/windows/object_handle.hpp"
#include "asio/windows/overlapped_handle.hpp"
#include "asio/windows/overlapped_ptr.hpp"
#include "asio/windows/random_access_handle.hpp"
#include "asio/windows/stream_handle.hpp"
#include "asio/write.hpp"
#include "asio/write_at.hpp"

#endif // ASIO_HPP
