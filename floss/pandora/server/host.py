# Copyright 2023 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Host grpc interface."""

import asyncio
import logging
from typing import AsyncGenerator
import uuid as uuid_module

from floss.pandora.floss import adapter_client
from floss.pandora.floss import advertising_client
from floss.pandora.floss import floss_enums
from floss.pandora.floss import scanner_client
from floss.pandora.floss import utils
from floss.pandora.server import bluetooth as bluetooth_module
from google.protobuf import any_pb2
from google.protobuf import empty_pb2
import grpc
from pandora import host_grpc_aio
from pandora import host_pb2


class HostService(host_grpc_aio.HostServicer):
    """Service to trigger Bluetooth Host procedures.

    This class implements the Pandora bluetooth test interfaces,
    where the meta class definition is automatically generated by the protobuf.
    The interface definition can be found in:
    https://cs.android.com/android/platform/superproject/+/main:external
    /pandora/bt-test-interfaces/pandora/host.proto
    """

    def __init__(self, server: grpc.aio.Server, bluetooth: bluetooth_module.Bluetooth):
        self.server = server
        self.bluetooth = bluetooth
        self.waited_connections = set()

    async def FactoryReset(self, request: empty_pb2.Empty, context: grpc.ServicerContext) -> empty_pb2.Empty:
        self.waited_connections = set()
        asyncio.create_task(self.server.stop(None))
        return empty_pb2.Empty()

    async def Reset(self, request: empty_pb2.Empty, context: grpc.ServicerContext) -> empty_pb2.Empty:
        self.waited_connections = set()
        self.bluetooth.reset()
        return empty_pb2.Empty()

    async def ReadLocalAddress(self, request: empty_pb2.Empty,
                               context: grpc.ServicerContext) -> host_pb2.ReadLocalAddressResponse:
        address = self.bluetooth.get_address()
        return host_pb2.ReadLocalAddressResponse(address=utils.address_to(address))

    async def Connect(self, request: host_pb2.ConnectRequest,
                      context: grpc.ServicerContext) -> host_pb2.ConnectResponse:

        class PairingObserver(adapter_client.BluetoothCallbacks, adapter_client.BluetoothConnectionCallbacks):
            """Observer to observe the bond state and the connection state."""

            def __init__(self, client: adapter_client, task):
                self.client = client
                self.task = task

            @utils.glib_callback()
            def on_bond_state_changed(self, status, address, state):
                if address != self.task['address']:
                    return

                if status != 0:
                    self.task['connect_device'].set_result(
                        (False, f'{address} failed to bond. Status: {status}, State: {state}'))
                    return

                if state == floss_enums.BondState.BONDED:
                    if not self.client.is_connected(self.task['address']):
                        logging.info('{address} calling connect_all_enabled_profiles')
                        if not self.client.connect_all_enabled_profiles(self.task['address']):
                            self.task['connect_device'].set_result(
                                (False, f'{self.task["address"]} failed on connect_all_enabled_profiles'))
                    else:
                        self.task['connect_device'].set_result((True, None))

            @utils.glib_callback()
            def on_ssp_request(self, remote_device, class_of_device, variant, passkey):
                address, _ = remote_device
                if address != self.task['address']:
                    return

                if variant == floss_enums.SspVariant.CONSENT:
                    self.client.set_pairing_confirmation(address,
                                                         True,
                                                         method_callback=self.on_set_pairing_confirmation)

            @utils.glib_callback()
            def on_set_pairing_confirmation(self, err, result):
                if err or not result:
                    self.task['connect_device'].set_result(
                        (False, f'Pairing confirmation failed: err: {err}, result: {result}'))

            @utils.glib_callback()
            def on_device_connected(self, remote_device):
                address, _ = remote_device
                if address != self.task['address']:
                    return

                if self.client.is_bonded(address):
                    self.task['connect_device'].set_result((True, None))

        address = utils.address_from(request.address)

        if not self.bluetooth.is_connected(address):
            try:
                connect_device = asyncio.Future()
                observer = PairingObserver(self.bluetooth.adapter_client, {
                    'connect_device': connect_device,
                    'address': address
                })
                name = utils.create_observer_name(observer)
                self.bluetooth.adapter_client.register_callback_observer(name, observer)

                if self.bluetooth.is_bonded(address):
                    self.bluetooth.connect_device(address)
                else:
                    if not self.bluetooth.create_bond(address, floss_enums.BtTransport.BR_EDR):
                        raise RuntimeError('Failed to call create_bond.')

                success, reason = await connect_device
                if not success:
                    raise RuntimeError(f'Failed to connect to the {address}. Reason: {reason}')
            finally:
                self.bluetooth.adapter_client.unregister_callback_observer(name, observer)

        cookie = any_pb2.Any(value=utils.address_to(request.address))
        return host_pb2.ConnectResponse(connection=host_pb2.Connection(cookie=cookie))

    async def WaitConnection(self, request: host_pb2.WaitConnectionRequest,
                             context: grpc.ServicerContext) -> host_pb2.WaitConnectionResponse:

        class ConnectionObserver(adapter_client.BluetoothConnectionCallbacks):
            """Observer to observe the connection state."""

            def __init__(self, task):
                self.task = task

            @utils.glib_callback()
            def on_device_connected(self, remote_device):
                address, _ = remote_device
                if address != self.task['address']:
                    return

                self.task['wait_connection'].set_result(address)

        if request.address is None:
            raise ValueError('Request address field must be set.')
        address = utils.address_from(request.address)

        if not self.bluetooth.is_connected(address) or address not in self.waited_connections:
            try:
                wait_connection = asyncio.Future()
                observer = ConnectionObserver({'wait_connection': wait_connection, 'address': address})
                name = utils.create_observer_name(observer)
                self.bluetooth.adapter_client.register_callback_observer(name, observer)

                await wait_connection
            finally:
                self.bluetooth.adapter_client.unregister_callback_observer(name, observer)
            self.waited_connection.add(address)

        cookie = any_pb2.Any(value=utils.address_to(address))
        return host_pb2.WaitConnectionResponse(connection=host_pb2.Connection(cookie=cookie))

    async def ConnectLE(self, request: host_pb2.ConnectLERequest,
                        context: grpc.ServicerContext) -> host_pb2.ConnectLEResponse:
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)  # type: ignore
        context.set_details('Method not implemented!')  # type: ignore
        raise NotImplementedError('Method not implemented!')

    async def Disconnect(self, request: host_pb2.DisconnectRequest, context: grpc.ServicerContext) -> empty_pb2.Empty:
        address = utils.address_from(request.connection.cookie.value)
        if self.bluetooth.is_connected(address):
            self.bluetooth.disconnect_device(address)
        return empty_pb2.Empty()

    async def WaitDisconnection(self, request: host_pb2.WaitDisconnectionRequest,
                                context: grpc.ServicerContext) -> empty_pb2.Empty:

        class ConnectionObserver(adapter_client.BluetoothConnectionCallbacks):
            """Observer to observe the connection state."""

            def __init__(self, task):
                self.task = task

            @utils.glib_callback()
            def on_device_disconnected(self, remote_device):
                address, _ = remote_device
                if address != self.task['address']:
                    return
                self.task['wait_disconnection'].set_result(address)

        if request.address is None:
            raise ValueError('Request address field must be set')
        address = utils.address_from(request.address)

        if self.bluetooth.is_connected(address):
            try:
                wait_disconnection = asyncio.Future()
                observer = ConnectionObserver({'wait_disconnection': wait_disconnection, 'address': address})
                name = utils.create_observer_name(observer)
                self.bluetooth.adapter_client.register_callback_observer(name, observer)
                await wait_disconnection
            finally:
                self.bluetooth.adapter_client.unregister_callback_observer(name, observer)

        return empty_pb2.Empty()

    async def Advertise(self, request: host_pb2.AdvertiseRequest,
                        context: grpc.ServicerContext) -> AsyncGenerator[host_pb2.AdvertiseResponse, None]:
        parameters = {
            'connectable': request.connectable,
            'scannable': True,
            'is_legacy': True,  # ROOTCANAL: Extended advertising ignored because the scanner is legacy.
            'is_anonymous': False,
            'include_tx_power': True,
            'primary_phy': 1,
            'secondary_phy': 1,
            'interval': request.interval,
            'tx_power_level': 127,  # 0x7f
            'own_address_type': -1,  # default
        }

        primary_phy = request.primary_phy
        if primary_phy == host_pb2.PRIMARY_1M:
            parameters['primary_phy'] = floss_enums.LePhy.PHY1M
        elif primary_phy == host_pb2.PRIMARY_CODED:
            parameters['primary_phy'] = floss_enums.LePhy.PHY_CODED

        secondary_phy = request.secondary_phy
        if secondary_phy == host_pb2.SECONDARY_NONE:
            parameters['secondary_phy'] = floss_enums.LePhy.INVALID
        elif secondary_phy == host_pb2.SECONDARY_1M:
            parameters['secondary_phy'] = floss_enums.LePhy.PHY1M
        elif secondary_phy == host_pb2.SECONDARY_2M:
            parameters['secondary_phy'] = floss_enums.LePhy.PHY2M
        elif secondary_phy == host_pb2.SECONDARY_CODED:
            parameters['secondary_phy'] = floss_enums.LePhy.PHY_CODED

        own_address_type = request.own_address_type
        if own_address_type in (host_pb2.PUBLIC, host_pb2.RESOLVABLE_OR_PUBLIC):
            parameters['own_address_type'] = floss_enums.OwnAddressType.PUBLIC
        elif own_address_type in (host_pb2.RANDOM, host_pb2.RESOLVABLE_OR_RANDOM):
            parameters['own_address_type'] = floss_enums.OwnAddressType.RANDOM

        # TODO: b/289480188 - Support more data and scan response data if needed.
        advertise_data = utils.advertise_data_from(request.data)

        class AdvertisingObserver(advertising_client.BluetoothAdvertisingCallbacks):
            """Observer to observe the advertising state."""

            def __init__(self, task):
                self.task = task

            @utils.glib_callback()
            def on_advertising_set_started(self, reg_id, advertiser_id, tx_power, status):
                if reg_id != self.task['reg_id']:
                    return

                if status is None or floss_enums.GattStatus(status) != floss_enums.GattStatus.SUCCESS:
                    logging.error('Failed to start advertising.')
                    advertiser_id = None
                self.task['start_advertising'].set_result(advertiser_id)

        class ConnectionObserver(adapter_client.BluetoothConnectionCallbacks):
            """Observer to observe all connections."""

            def __init__(self, loop: asyncio.AbstractEventLoop, task):
                self.loop = loop
                self.task = task

            @utils.glib_callback()
            def on_device_connected(self, remote_device):
                address, _ = remote_device
                asyncio.run_coroutine_threadsafe(self.task['connections'].put(address), self.loop)

        started_ids = []
        observers = []
        try:
            if request.connectable:
                connections = asyncio.Queue()
                observer = ConnectionObserver(asyncio.get_running_loop(), {'connections': connections})
                name = utils.create_observer_name(observer)
                self.bluetooth.adapter_client.register_callback_observer(name, observer)
                observers.append((name, observer))

            while True:
                if not self.bluetooth.advertising_client.active_advs:
                    reg_id = self.bluetooth.start_advertising_set(parameters, advertise_data, None, None, None, 0, 0)

                    advertising_request = {
                        'start_advertising': asyncio.get_running_loop().create_future(),
                        'reg_id': reg_id
                    }
                    observer = AdvertisingObserver(advertising_request)
                    name = utils.create_observer_name(observer)
                    self.bluetooth.advertising_client.register_callback_observer(name, observer)
                    observers.append((name, observer))

                    advertiser_id = await asyncio.wait_for(advertising_request['start_advertising'], timeout=5)
                    started_ids.append(advertiser_id)

                if not request.connectable:
                    await asyncio.sleep(1)
                    continue

                logging.info('Advertise: Wait for LE connection...')
                address = await connections.get()
                logging.info(f'Advertise: Connected to {address}')

                cookie = any_pb2.Any(value=utils.address_to(address))
                yield host_pb2.AdvertiseResponse(connection=host_pb2.Connection(cookie=cookie))

                # Wait a small delay before restarting the advertisement.
                await asyncio.sleep(1)
        finally:
            for name, observer in observers:
                self.bluetooth.adapter_client.unregister_callback_observer(name, observer)
                self.bluetooth.advertising_client.unregister_callback_observer(name, observer)

            for started in started_ids:
                self.bluetooth.stop_advertising_set(started)

    async def Scan(self, request: host_pb2.ScanRequest,
                   context: grpc.ServicerContext) -> AsyncGenerator[host_pb2.ScanningResponse, None]:

        class ScanObserver(scanner_client.BluetoothScannerCallbacks):
            """Observer to observer the scan state and scan results."""

            def __init__(self, loop: asyncio.AbstractEventLoop, task):
                self.loop = loop
                self.task = task

            @utils.glib_callback()
            def on_scanner_registered(self, uuid, scanner_id, status):
                uuid = uuid_module.UUID(bytes=bytes(uuid))
                if uuid != self.task['uuid']:
                    return

                if floss_enums.GattStatus(status) != floss_enums.GattStatus.SUCCESS:
                    logging.error('Failed to register scanner! uuid: {uuid}')
                    scanner_id = None
                self.task['register_scanner'].set_result(scanner_id)

            @utils.glib_callback()
            def on_scan_result(self, scan_result):
                asyncio.run_coroutine_threadsafe(self.task['scan_results'].put(scan_result), self.loop)

        scanner_id = None
        name = None
        observer = None
        try:
            uuid = self.bluetooth.register_scanner()
            scan = {
                'register_scanner': asyncio.get_running_loop().create_future(),
                'uuid': uuid,
                'scan_results': asyncio.Queue()
            }
            observer = ScanObserver(asyncio.get_running_loop(), scan)
            name = utils.create_observer_name(observer)
            self.bluetooth.scanner_client.register_callback_observer(name, observer)

            scanner_id = await asyncio.wait_for(scan['register_scanner'], timeout=10)

            self.bluetooth.start_scan(scanner_id)
            while True:
                scan_result = await scan['scan_results'].get()

                response = host_pb2.ScanningResponse()
                response.tx_power = scan_result['tx_power']
                response.rssi = scan_result['rssi']
                response.sid = scan_result['advertising_sid']
                response.periodic_advertising_interval = scan_result['periodic_adv_int']

                if scan_result['primary_phy'] == floss_enums.LePhy.PHY1M:
                    response.primary_phy = host_pb2.PRIMARY_1M
                elif scan_result['primary_phy'] == floss_enums.LePhy.PHY_CODED:
                    response.primary_phy = host_pb2.PRIMARY_CODED
                else:
                    pass

                if scan_result['secondary_phy'] == floss_enums.LePhy.INVALID:
                    response.secondary_phy = host_pb2.SECONDARY_NONE
                elif scan_result['secondary_phy'] == floss_enums.LePhy.PHY1M:
                    response.secondary_phy = host_pb2.SECONDARY_1M
                elif scan_result['secondary_phy'] == floss_enums.LePhy.PHY2M:
                    response.secondary_phy = host_pb2.SECONDARY_2M
                elif scan_result['secondary_phy'] == floss_enums.LePhy.PHY_CODED:
                    response.secondary_phy = host_pb2.SECONDARY_CODED

                address = bytes.fromhex(scan_result['address'].replace(':', ''))
                if scan_result['addr_type'] == floss_enums.BleAddressType.BLE_ADDR_PUBLIC:
                    response.public = address
                elif scan_result['addr_type'] == floss_enums.BleAddressType.BLE_ADDR_RANDOM:
                    response.random = address
                elif scan_result['addr_type'] == floss_enums.BleAddressType.BLE_ADDR_PUBLIC_ID:
                    response.public_identity = address
                elif scan_result['addr_type'] == floss_enums.BleAddressType.BLE_ADDR_RANDOM_ID:
                    response.random_static_identity = address

                # TODO: b/289480188 - Support more data if needed.
                mode = host_pb2.NOT_DISCOVERABLE
                if scan_result['flags'] & (1 << 0):
                    mode = host_pb2.DISCOVERABLE_LIMITED
                elif scan_result['flags'] & (1 << 1):
                    mode = host_pb2.DISCOVERABLE_GENERAL
                else:
                    mode = host_pb2.NOT_DISCOVERABLE
                response.data.le_discoverability_mode = mode
                yield response
        finally:
            if scanner_id is not None:
                self.bluetooth.stop_scan(scanner_id)
            if name is not None and observer is not None:
                self.bluetooth.scanner_client.unregister_callback_observer(name, observer)

    async def Inquiry(self, request: empty_pb2.Empty,
                      context: grpc.ServicerContext) -> AsyncGenerator[host_pb2.InquiryResponse, None]:

        class InquiryResultObserver(adapter_client.BluetoothCallbacks):
            """Observer to observe all inquiry results."""

            def __init__(self, loop: asyncio.AbstractEventLoop, task):
                self.loop = loop
                self.task = task

            @utils.glib_callback()
            def on_device_found(self, remote_device):
                address, _ = remote_device
                asyncio.run_coroutine_threadsafe(self.task['inquiry_results'].put(address), self.loop)

        class DiscoveryObserver(adapter_client.BluetoothCallbacks):
            """Observer to observe discovery state."""

            def __init__(self, task):
                self.task = task

            @utils.glib_callback()
            def on_discovering_changed(self, discovering):
                if discovering == self.task['discovering']:
                    self.task['start_inquiry'].set_result(discovering)

        observers = []
        try:
            if not self.bluetooth.is_discovering():
                inquriy = {'start_inquiry': asyncio.Future(), 'discovering': True}
                observer = DiscoveryObserver(inquriy)
                name = utils.create_observer_name(observer)
                self.bluetooth.adapter_client.register_callback_observer(name, observer)
                observers.append((name, observer))

                self.bluetooth.start_discovery()
                await asyncio.wait_for(inquriy['start_inquiry'], timeout=10)

            inquiry_results = asyncio.Queue()
            observer = InquiryResultObserver(asyncio.get_running_loop(), {'inquiry_results': inquiry_results})
            name = utils.create_observer_name(observer)
            self.bluetooth.adapter_client.register_callback_observer(name, observer)
            observers.append((name, observer))

            while True:
                address = await inquiry_results.get()
                yield host_pb2.InquiryResponse(address=utils.address_to(address))
        finally:
            self.bluetooth.stop_discovery()
            for name, observer in observers:
                self.bluetooth.adapter_client.unregister_callback_observer(name, observer)

    async def SetDiscoverabilityMode(self, request: host_pb2.SetDiscoverabilityModeRequest,
                                     context: grpc.ServicerContext) -> empty_pb2.Empty:
        mode = request.mode
        duration = 600  # Sets general, limited default to 60s. This is unused by the non-discoverable mode.
        self.bluetooth.set_discoverable(mode, duration)
        return empty_pb2.Empty()

    async def SetConnectabilityMode(self, request: host_pb2.SetConnectabilityModeRequest,
                                    context: grpc.ServicerContext) -> empty_pb2.Empty:
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)  # type: ignore
        context.set_details('Method not implemented!')  # type: ignore
        raise NotImplementedError('Method not implemented!')
