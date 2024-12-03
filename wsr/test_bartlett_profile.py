#!/usr/bin/env python3
import pickle
import argparse
from pathlib import Path
import logging
import base64

import numpy as np
from matplotlib import cm
from scipy.interpolate import interp1d
from scipy.ndimage.filters import maximum_filter
from scipy.signal import find_peaks
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid

import pywsr
from plot_displacement import plot_displacement
from plot_channel_data import plot_channel_data


LOG = logging.getLogger(__name__)
PROFILE3D = False
TOP_N = 3

def bartlett_profile_files(csi_fpath: Path, odometry_fpath: Path, mac: str, b64=False, show=False, name=None, half=False, last_n=0, emitter_origin=False):
    with csi_fpath.open("rb") as fd:
        if b64:
            # raw_data = fd.read()
            raw_data = pickle.load(fd)
            data = [pickle.loads(base64.b64decode(pkl.data)) for pkl in raw_data]
            data = [d['data'][0] for d in data]
        else:
            data = pickle.load(fd)
    with odometry_fpath.open("rb") as fd:
        odometry = pickle.load(fd)
        # print(odometry.shape)
        if not isinstance(odometry, np.ndarray):
            odometry = [[od.pose.pose.position.y, od.pose.pose.position.x, od.pose.pose.position.z] for od in odometry]
            odometry = np.array(odometry)
            odometry.reshape((-1, 3))
        odometry[:,2] = 0
        # odometry = odometry - odometry[0]
        # else:
        #     odometry_tmp = odometry[:,0]
        #     odometry[:,0] = odometry[:,2]
        #     odometry[:,2] = odometry_tmp

    if last_n > 0:
        data = data[-last_n:]
    return bartlett_profile(data, odometry, mac, show, name=name, half=half)

def inner_angle(a, b):
    ''' return in radians '''
    return np.arccos(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))

# def relative_angle(position, orientation):
#     angle_pos = np.arctan2(position[0], position[1])
#     angle_or = np.arctan2(orientation[0], orientation[1])
#     angle_tot = angle_pos - angle_or + np.pi
#     if angle_tot < 0:
#         print("adding 2pi")
#         angle_tot += np.pi *2
#     elif angle_tot > np.pi*2:
#         print("removing 2pi")
#         angle_tot -= np.pi *2
#     return angle_tot

def truth_angle(tx_loc, rx_loc):
    ''' returns degrees
    determined by looking at wsr example data

    angle is degrees from x axis. above x is positive, below x is negative, negative along x is 180 degrees

    y
    ^
    |   .> +
    |  .
    | .
    |. +θ
    O----------->x
    .  -θ
     .
      .>
    question: does orentation matter? i dont think so. its not like the frame changing
    '''
    xax = np.array([1, 0, 0])
    yax = np.array([0, 1, 0])
    rx_to_tx = tx_loc - rx_loc
    angle = np.degrees(inner_angle(rx_to_tx, xax))
    if np.dot(yax, rx_to_tx) > 0:
        return angle
    else:
        return -angle



def diff_360(a, b):
    tmp = a-b
    if tmp > 180:
        tmp -= 360
    elif tmp < -180:
        tmp += 360
    return tmp

def profile_variance(profile, phi_idx, theta_idx):
    # todo: get these from config file
    ntheta = 90
    nphi = 360
    sumf = np.sum(profile)
    sigma_f, sigma_n, temp1, temp2 = 0, 0, 0, 0
    for row in range(nphi):
        for col in range(ntheta):
            temp1 = diff_360(row, phi_idx)**2
            temp2 = (col - theta_idx)**2
            sigma_f += (temp1 + temp2) * profile[row, col] / sumf
            sigma_n += (temp1 + temp2) * sumf / (ntheta * nphi)
    return sigma_f / sigma_n

def bartlett_profile(csi_data, odometry, mac: str, show=False, name=None, half=False):

    # lamp somehow got 2 tx at one point?
    print(f'shape of csi mat [0]: {csi_data[0]["csi_matrix"].shape}')

    data = [d for d in csi_data if d['csi_matrix'].shape == (52, 2, 1)]

    macs = list(set([d['header']['source_mac_string'] for d in data]))
    print(f"found macs in data: {macs}")
    if mac is not None:
        data = [d for d in  data if d['header']['source_mac_string'] == mac]
    print(f"len data after mac filter: {len(data)}")
    if not data:
        print("bailing...")
        return 0, 0
    if half:
        data = data[:len(data) // 2]

    csi_mat = np.array([msg['csi_matrix'] for msg in data]).squeeze(axis=3)
    # csi_mat = csi_mat[:, ::-1, :] # flip csi

    h_data = csi_mat[:, :, 0] * np.conj(csi_mat[:, :, 1])
    # h_data = csi_mat[:, :, 0] * csi_mat[:, :, 1]
    if half:
        x = np.arange(0, odometry.shape[0] // 2)
        interp_od = interp1d(x, odometry[:len(odometry) // 2, :], axis=0)
    else:
        x = np.arange(0, odometry.shape[0])
        interp_od = interp1d(x, odometry - odometry[0], axis=0)
        # interp_od = interp1d(x, odometry, axis=0)
        interp_od_abs = interp1d(x, odometry, axis=0)
    pose = interp_od(np.linspace(0, x[-1], csi_mat.shape[0]))
    pose_abs = interp_od_abs(np.linspace(0, x[-1], csi_mat.shape[0]))


    LOG.info(f"{h_data.shape=},{pose.shape=}")
    profile = pywsr.compute_profile_bartlett(0, h_data, pose)
    LOG.info(profile.shape)
    z = profile.flatten()
    ind = np.unravel_index(np.argmax(profile, axis=None), profile.shape)
    LOG.info("variance=%s", profile_variance(profile, *ind))

    if show:
        import plotly.graph_objects as go
        import plotly
        import matplotlib.pyplot as plt
        from plotly.subplots import make_subplots


        sh_0, sh_1 = profile.shape
        x, y = np.linspace(-180, 180, sh_0), np.linspace(0, 90, sh_1)
        LOG.info("xshape=%s, yshape=%s, profileshape=%s", x.shape, y.shape, profile.shape)
        name = name or ""
        fig = make_subplots(rows=2, cols=2, horizontal_spacing = 0.05, specs=[
            # [{"type":"scene", "rowspan":2}, {"type":"xy"}],
            [{"type":"scene"}, {"type":"scene"} if PROFILE3D else {"type":"xy"} ],
            [{"type":"xy"}, {"type":"xy"}]],
                            subplot_titles=("Displacement", f"AoA Profile (2D) [σ={profile_variance(profile, *ind)}]", "Amplitude over time", "Phase over time"))

        # fig.add_trace(go.Surface(x=y, y=x, z=profile), row=1, col=1)

        fig.add_trace(go.Scatter3d(x=pose_abs[:,0], y=pose_abs[:,1], z=pose_abs[:,2]), row=1, col=1)
        fig.add_trace(go.Scatter3d(x=[pose_abs[0,0]], y=[pose_abs[0,1]], z=[pose_abs[0,2]], surfacecolor="red"), row=1, col=1)
        fig.add_trace(go.Scatter3d(x=[0], y=[0], z=[0], surfacecolor="green"), row=1, col=1)
        fig.add_trace(go.Scatter3d(x=[0, pose_abs[-1,0]], y=[0, pose_abs[-1,1]], z=[0, pose_abs[-1,2]], mode="lines", surfacecolor="blue"), row=1, col=1)

        times = [d['header']['ftm_clock'] for d in data]
        rss1 = [-1 * d['header']['rssi1'] for d in data]
        rss2 = [-1 * d['header']['rssi2'] for d in data]
        fig.add_trace(go.Scatter(x=times, y=np.unwrap(np.angle(csi_mat[:, 30, 0])), ), row=2, col=2)
        fig.update_xaxes(title_text="Time (clock sample)", row=2, col=2)
        fig.update_yaxes(title_text="Unwrapped Phase (radians)", row=2, col=2)

        if PROFILE3D:
            fig.add_trace(go.Surface(x=y, y=x, z=profile), row=1, col=2)
        else:
            peakx, peak_data = find_peaks(profile[:, 0], height=100e-6, distance=10)
            fig.add_trace(go.Scatter(x=x, y=profile[:, 0]), row=1, col=2)
            fig.add_trace(go.Scatter(x=peakx-180, y=peak_data['peak_heights'], marker=dict(size=10), mode="markers"), row=1, col=2)


            truth_angle_start_deg = truth_angle(np.array([0,0,0]), pose_abs[0,:])
            truth_angle_end_deg = truth_angle(np.array([0,0,0]), pose_abs[-1,:])
            fig.add_shape(
                go.layout.Shape(type='line', xref='x', yref='paper',
                                x0=truth_angle_start_deg, y0=0, x1=truth_angle_start_deg, y1=np.max(profile[:,0]), line={'dash': 'dot'}),
                row=1, col=2)
            fig.add_shape(
                go.layout.Shape(type='line', xref='x', yref='paper',
                                x0=truth_angle_end_deg, y0=0, x1=truth_angle_end_deg, y1=np.max(profile[:,0]), line={'dash': 'dash'}),
                row=1, col=2)


            peak_sort_inds = np.argsort(peak_data['peak_heights'])
            peakx -= 180
            for ii in range(TOP_N):
                if len(peak_sort_inds) <= ii:
                    break
                print(f"Top [{ii}] startdiff={truth_angle_start_deg - peakx[peak_sort_inds[ii]]} enddiff={truth_angle_end_deg - peakx[peak_sort_inds[ii]]}")
            print("End Top")
            # fig.add_vline(x=truth_angle_start_deg, row=1, col=2)
            # fig.add_vline(x=truth_angle_end_deg, row=1, col=2)


        fig.update_xaxes(title_text=f"Azimuth AoA (degrees)", row=1, col=2)
        fig.update_yaxes(title_text="Magnitude", row=1, col=2)


        fig.update_scenes(xaxis_title_text="X (meters)",
                          yaxis_title_text="Y (meters)",
                          zaxis_title_text="Z (meters)")
        fig.update_scenes(aspectmode="data")
        fig.update_layout(title=name)

        # height determined through experiment

        fig.add_trace(go.Scatter(x=times, y=rss1), row=2, col=1)
        fig.add_trace(go.Scatter(x=times, y=rss2), row=2, col=1)

        # fig.show()
        plotly.offline.plot(fig, filename=f"{name}.html")


    azimuth_deg, elevation_deg = ind

    return azimuth_deg, elevation_deg, profile[ind]



if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("csi_pkls", type=Path, nargs="+", help="csi pickle files, poses assumed to be same but odometry instead of csi")
    # ap.add_argument("pose_pkl", type=Path)
    ap.add_argument("--mac", default=None)
    ap.add_argument("--b64", action="store_true")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--half", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--last-n", default=0, type=int)
    ap.add_argument("--emitter-origin", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)
    for ii, csi_pkl in enumerate(args.csi_pkls):
        robot,_,timestamp = csi_pkl.name.split("_")
        name = f"[{ii}] {robot} {timestamp}"
        pose_pkl = csi_pkl.parent / csi_pkl.name.replace("csi", "odometry")
        try:
            az, el, z = bartlett_profile_files(csi_pkl, pose_pkl, args.mac, args.b64, args.show, name=name, half=args.half, last_n=args.last_n, emitter_origin=args.emitter_origin)
            print(f"{az=} {el=} {z=}")
        except ValueError:
            pass

    #     if args.show:
    #         plot_displacement(pose_pkl, name=name)
    #         plot_channel_data(csi_pkl, 0, 0, args.mac, args.b64, name=name)
    # if args.show:
    #     plt.show()
