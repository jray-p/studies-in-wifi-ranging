
from pathlib import Path
import numpy as np
import struct
from binascii import b2a_hex
import pickle

import base64

# WSR format
def parse_wsr_record(rec_bytes: bytes):
    timestamp_low = rec_bytes[0] + (rec_bytes[1] << 8) + (rec_bytes[2] << 16) + (rec_bytes[3] << 24)

    bfee_count = rec_bytes[4] + (rec_bytes[5] << 8);
    Nrx = rec_bytes[8];
    Ntx = rec_bytes[9];
    rssi_a = rec_bytes[10];
    rssi_b = rec_bytes[11];
    rssi_c = rec_bytes[12];
    noise = rec_bytes[13];
    agc = rec_bytes[14];
    antenna_sel = rec_bytes[15];
    rec_len = rec_bytes[16] + (rec_bytes[17] << 8);
    fake_rate_n_flags = rec_bytes[18] + (rec_bytes[19] << 8);
    tv_sec = rec_bytes[20] + (rec_bytes[21] << 8) + (rec_bytes[22] << 16) + (rec_bytes[23] << 24);
    tv_usec = rec_bytes[24] + (rec_bytes[25] << 8) + (rec_bytes[26] << 16) + (rec_bytes[27] << 24);
    # unsigned char *mac = &rec_bytes[28];
    mac = b2a_hex(rec_bytes[28:34], ":").decode("utf-8")
    frame_count = rec_bytes[34] + (rec_bytes[35] << 8) + (rec_bytes[36] << 16) + (rec_bytes[37] << 24);
    calc_len = (30 * (Nrx * Ntx * 8 * 2 + 3) + 7) // 8;
    # print(rec_len, calc_len, frame_count, tv_sec + tv_usec*1e-6)
    payload = rec_bytes[38:]

    index=0
    csi = np.zeros((30, Nrx*Ntx), dtype=complex)
    for ii in range(30):
        index+=3
        remainder = index % 8
        for jj in range(Nrx * Ntx):

            # tmp1 = (payload[index // 8] >> remainder) | (payload[index//8+1] << (8-remainder))
            # tmp2 = (payload[(index // 8)+1] >> remainder) | (payload[index//8+2] << (8-remainder))
            tmp1 = struct.unpack("b", struct.pack("B", ((payload[index // 8] >> remainder) | (payload[index//8+1] << (8-remainder)) & 0xff)))[0]
            tmp2 = struct.unpack("b", struct.pack("B", ((payload[(index // 8)+1] >> remainder) | (payload[index//8+2] << (8-remainder)) & 0xff)))[0]
            index+=16
            csi[ii, jj] = tmp1 + 1j*tmp2

    return {
        "header": {
            "ftm_clock": tv_sec + tv_usec*1e-6,
            "source_mac_string":mac,
            "rssi1": rssi_a,
            "rssi2": rssi_b,
            "rssi3": rssi_c,
        },
        "csi_matrix":csi,
    }

def read_wsr_csi(csi_path: Path) -> list:
    csi_packets = list()
    num_parsed = 0
    with csi_path.open("rb") as fd:
        while True:
            try:
                fd.read(1) # skip first byte i guess
                field_size = struct.unpack("B", fd.read(1))[0] - 1
                field_code = struct.unpack("B", fd.read(1))[0]
                if field_code == 187:

                    # print(f"got record of size {field_size}")
                    # parse record
                    record_data = fd.read(field_size)
                    try:
                        csi_packets.append(parse_wsr_record(record_data))
                        num_parsed+=1
                    except IndexError:
                        pass
                else:
                    # print(f"skipping {field_size} bytes")
                    fd.read(field_size)
            except struct.error as err:
                print(err)
                break
    print(f"{num_parsed=} from {csi_path}")
    return csi_packets
# end WSR format

def parse_csi_file(fpath, mac=None, b64=True):
    ''' return csi_matrix and headers
    csi_mat is (Np,subchan,Nrx)
      where Np is number packets,
      Nrx is number rx
      subchan is number of subchannels (52 or 30)
    headers is a dict with keys:
      macs: list of macs
      times: list of times
      rssi1: list of rssi1
      rssi2: list of rssi2
    '''
    csishape = (52,2,1)
    if fpath.suffix == ".dat":
        data = read_wsr_csi(fpath)
        csishape = (30,3)
    else:
        with fpath.open("rb") as fd:
            if b64:
                raw_data = pickle.load(fd)
                data = [pickle.loads(base64.b64decode(pkl.data)) for pkl in raw_data]
                data = [d['data'][0] for d in data]
            else:
                data = pickle.load(fd)

    # filter for correct shape
    data = [d for d in data if d['csi_matrix'].shape == csishape]
    if mac:
        data = [d for d in data if d['header']['source_mac_string'].lower() == mac.lower()]
    if fpath.suffix == ".dat":
        csi_mat = np.array([msg['csi_matrix'] for msg in data])
    else:
        csi_mat = np.array([msg['csi_matrix'] for msg in data]).squeeze(axis=3)
    header = {
        "mac": [d['header']['source_mac_string'].lower() for d in data],
        "times": [d['header']['ftm_clock'] for d in data],
        "rssi1": [d['header']['rssi1'] for d in data],
        "rssi2": [d['header']['rssi2'] for d in data],
        "rssi3": [d['header']['rssi3'] for d in data] if "rssi3" in data[0]['header'] else [],
    }
    return csi_mat, header


datcsi, datheader = parse_csi_file(Path("/home/jray/WSR_Project/WSR-Toolbox-cpp/data/Sample_data/single_antenna/csi_rx_2021-06-27_202550.dat"))
# pklcsi, pklheader = parse_csi_file(Path("/home/jray/src/Wifi_SLAM/wsr/captures/left0/bonnie_csi_1722027569.1998804.pkl"), mac="00:16:ea:12:34:58")
