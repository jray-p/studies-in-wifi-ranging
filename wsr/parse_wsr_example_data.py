'''
parse the wsr example data files
'''

from pathlib import Path
import numpy as np
import struct
from binascii import b2a_hex


def parse_record(rec_bytes: bytes):
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
    print(rec_len, calc_len, frame_count, tv_sec + tv_usec*1e-6)
    payload = rec_bytes[38:]

    index=0
    csi = np.zeros((30, Nrx*Ntx), dtype=complex)
    for ii in range(30):
        index+=3
        remainder = index % 8
        for jj in range(Nrx * Ntx):
            tmp1 = (payload[index // 8] >> remainder) | (payload[index//8+1] << (8-remainder))
            tmp2 = (payload[index // 8+1] >> remainder) | (payload[index//8+2] << (8-remainder))
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
                        csi_packets.append(parse_record(record_data))
                        num_parsed+=1
                    except IndexError:
                        pass
                else:
                    # print(f"skipping {field_size} bytes")
                    fd.read(field_size)
            except struct.error:
                break
    print(f"{num_parsed=} from {csi_path}")
    return csi_packets



csi_data = read_wsr_csi(Path("/home/jray/WSR_Project/WSR-Toolbox-cpp/data/Sample_data/single_antenna/csi_rx_2021-06-27_202550.dat"))
