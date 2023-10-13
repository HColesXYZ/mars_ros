from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import find_peaks
from tsplotter import *

def butterworth(signal, cutoff_freq = 2, sample_rate = 250, order=2):
    # Calculate the normalized cutoff frequency
    normalized_cutoff_freq = cutoff_freq / (0.5 * sample_rate)
    
    # Design the Butterworth filter
    b, a = butter(N=order, Wn=normalized_cutoff_freq, btype='low', analog=False)
    
    # Apply the filter to the signal using filtfilt
    filtered_signal = filtfilt(b, a, signal)
    
    return filtered_signal

def get_inflections(imu, opti):
    imu_mins, _ = find_peaks(-imu['an_filt'], distance=200, height=-8)
    opti_maxs, _ = find_peaks(opti['y'], distance=200, height=0.9)

    imu_maxs, _ = find_peaks(imu['an_filt'], distance=200, height=13)
    opti_mins, _ = find_peaks(-opti['y'], distance=200, height=[-0.85, -0.75])

    imu_points = np.sort(np.concatenate((imu_maxs, imu_mins)))
    opti_points = np.sort(np.concatenate((opti_maxs, opti_mins)))

    return imu_points, opti_points
    #return imu_maxs, opti_mins
    #return imu_mins, opti_maxs

def plot_data(imu, opti, imu_points, opti_points):
    _, ax1 = plt.subplots(1, 1, sharex=True)

    # plot the negative of the accel values for ease of comparison
    ax1.plot(imu['time'], -imu['an'], label='- Acceleration', color='red')
    ax1.plot(imu['time'], -imu['an_filt'], label='- Kalman', color='black')
    ax1.plot(imu['time'][imu_points], -imu['an_filt'][imu_points], 'go', label='Peaks')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Accelerometer (m/s^2)')
    ax1.legend(loc='upper left')

    ax2 = ax1.twinx()
    ax2.plot(opti['time'], opti['y'], label='Y Position')
    ax2.plot(opti['time'][opti_points], opti['y'][opti_points], 'mo', label='Peaks')
    ax2.set_ylabel('Y Position (m)')
    ax2.legend(loc='upper right')

    plt.show()

def calculate_mean_std(imu, opti, imu_peaks, opti_peaks):
    values1 = imu['time'][imu_peaks].tolist()
    values2 = opti['time'][opti_peaks].tolist()
    sub = [a - b for a, b in zip(values1, values2)]
    mean = np.mean(sub)
    std_dev = np.std(sub)
    print(f'OptiTrack, IMU offset: {mean.round(5)}s with {(std_dev * 1000).round(3)}ms std')
    return mean, std_dev

def main():
    folder_path = 'src/mars_ros/tools/data/shake_test_09-08-23'
    _, opti, imu = get_files(folder_path, do_ts=False)

    imu['an'] = np.sqrt(imu['ax']**2 + imu['ay']**2 + imu['az']**2)
    imu['an_filt'] = butterworth(imu['an'])

    imu_peaks, opti_peaks = get_inflections(imu, opti)

    mean, _ = calculate_mean_std(imu, opti, imu_peaks, opti_peaks)
    opti['time'] = opti['time'] + mean

    plot_data(imu, opti, imu_peaks, opti_peaks)

if __name__ == "__main__":
    main()