"""Generate an explicit, single-network Cyclone DDS profile."""
import argparse
import ipaddress
import json
from pathlib import Path
import shlex
import subprocess
import xml.etree.ElementTree as ET


def render(config, profile, role, interface=None, local_address=None):
    selected = config['profiles'][profile]
    local = selected[role]
    peer = selected['laptop' if role == 'pi' else 'pi']['address']
    subnet = ipaddress.ip_network(selected['subnet'])
    address = local_address or local['address']
    if ipaddress.ip_address(address) not in subnet or ipaddress.ip_address(peer) not in subnet:
        raise ValueError('Addresses must belong to the selected profile subnet')
    iface = interface or local['interface']
    if iface == 'auto':
        raise ValueError('Resolve the local interface first')
    root = ET.Element('CycloneDDS', xmlns='https://cdds.io/config')
    domain = ET.SubElement(root, 'Domain', Id=str(config['domain_id']))
    general = ET.SubElement(domain, 'General')
    interfaces = ET.SubElement(general, 'Interfaces')
    ET.SubElement(interfaces, 'NetworkInterface', name=iface, address=address)
    ET.SubElement(general, 'AllowMulticast').text = 'false'
    discovery = ET.SubElement(domain, 'Discovery')
    ET.SubElement(discovery, 'ParticipantIndex').text = 'auto'
    ET.SubElement(discovery, 'MaxAutoParticipantIndex').text = '99'
    peers = ET.SubElement(discovery, 'Peers')
    # Self peer discovers other processes on this host without multicast.
    for value in (address, peer): ET.SubElement(peers, 'Peer', Address=value)
    return ET.tostring(root, encoding='unicode')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--profile', choices=('hotspot','tailscale'), default='hotspot')
    parser.add_argument('--role', choices=('pi','laptop'), required=True)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    profile = config['profiles'][args.profile]
    local = profile[args.role]
    other = profile['laptop' if args.role == 'pi' else 'pi']['address']
    route = json.loads(subprocess.check_output(['ip','-j','route','get',other], text=True))[0]
    iface = route['dev'] if local['interface'] == 'auto' else local['interface']
    address = route.get('prefsrc', route.get('src'))
    if route['dev'] != iface or not address:
        parser.error('Selected interface does not route to the peer; fix network configuration')
    if ipaddress.ip_address(address) not in ipaddress.ip_network(profile['subnet']):
        parser.error('Peer route uses '+str(address)+', outside '+profile['subnet']+'. Inspect hotspot routing, WSL interface mirroring, and firewall rules; no automatic Tailscale fallback.')
    # When DHCP changes, advertise the actual local source and require updating the peer profile.
    if address != local['address']:
        parser.error('Local source is '+address+'; update this role address in network.json on BOTH hosts')
    xml = render(config, args.profile, args.role, iface, address)
    args.output.mkdir(parents=True, exist_ok=True)
    uri = args.output.resolve()/'cyclonedds.xml'
    uri.write_text(xml)
    values = {'RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp', 'ROS_DOMAIN_ID':str(config['domain_id']), 'CYCLONEDDS_URI':str(uri),
              'ROS_AUTOMATIC_DISCOVERY_RANGE':'SYSTEM_DEFAULT'}
    (args.output/'ros.env').write_text(''.join(k+'='+v+'\n' for k,v in values.items()))
    (args.output/'ros.sh').write_text(''.join('export '+k+'='+shlex.quote(v)+'\n' for k,v in values.items()))
    print('Generated', args.profile, args.role, 'via', iface, address, '->', other)
    print('source', args.output/'ros.sh')


if __name__ == '__main__': main()
