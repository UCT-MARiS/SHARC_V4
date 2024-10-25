import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import welch, firwin, lfilter, freqz
from scipy.optimize import curve_fit
import scipy.stats as stats
import os
from matplotlib.ticker import MultipleLocator

sampleRate = 1  # samples per second
duration = 1124  # seconds
length = sampleRate*duration
plotAll = False
scaling = "density" # density or spectrum

# simulate velocity waveform
def generate_wave(params, duration, sample_rate):
    """
    Generate a linear combination of sine waves based on the given parameters.
    
    Parameters:
    - params: n-dimensional numpy array with columns [amplitude, frequency, phase]
    - duration: Duration of the time series in seconds
    - sample_rate: Number of samples per second
    
    Returns:
    - time: Time array
    - combined_wave: Combined wave array
    """
    # Create a time array
    time = np.linspace(0, duration, int(duration * sample_rate))
    
    # Initialize the combined wave array
    combined_wave = np.zeros_like(time)
    
    # Loop through each set of parameters and generate the sine wave
    for amplitude, frequency, phase in params:
        sine_wave = amplitude * np.sin((2 * np.pi * frequency * time)+ phase)
        combined_wave += sine_wave
    
    return time, combined_wave

# chi-squared distribution
def calculate_psd_confidence_intervals(psd, df, confidence_level=0.90):
    """
    Calculate the lower and upper confidence intervals for a given PSD.
    
    Parameters:
    psd (array): Power Spectral Density values.
    df (int): Degrees of freedom.
    confidence_level (float): Confidence level for the intervals (default is 0.90).
    
    Returns:
    tuple: Lower and upper confidence intervals.
    """
    alpha = 1 - confidence_level
    chi2_lower = stats.chi2.ppf(alpha / 2, df)
    chi2_upper = stats.chi2.ppf(1 - alpha / 2, df)
    
    psd_lower = psd * df / chi2_upper
    psd_upper = psd * df / chi2_lower
    
    return psd_lower, psd_upper

# Function to apply FIR bandpass filter
def apply_fir_bandpass_filter(data, sample_rate, lowcut, highcut, numtaps=64):
    nyquist = 0.5 * sample_rate
    low = lowcut / nyquist
    high = highcut / nyquist
    fir_coefficients = firwin(numtaps, [low, high], pass_zero=False, window='hann')
    filtered_data = lfilter(fir_coefficients, 1.0, data)
    #print(fir_coefficients)

    if plotAll:
        w, h = freqz(fir_coefficients, worN=512)
        plt.figure(figsize=(8, 6))
        plt.plot(0.5 * sample_rate * w / np.pi, 20*np.log10(np.abs(h)), 'b', linewidth=0.5)
       # plt.title('Frequency Response of the FIR Bandpass Filter')
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Gain (dB)')
        plt.grid()

    return filtered_data, fir_coefficients

# Function to apply FIR low-pass filter
def apply_fir_lowpass_filter(data, sample_rate, cutoff, numtaps=32, plotAll=True):
    nyquist = 0.5 * sample_rate
    normalized_cutoff = cutoff / nyquist
    fir_coefficients = firwin(numtaps, normalized_cutoff, pass_zero=True, window='hann')
    filtered_data = lfilter(fir_coefficients, 1.0, data)

    if plotAll:
        w, h = freqz(fir_coefficients, worN=512)
        plt.figure(figsize=(8, 6))
        plt.plot(0.5 * sample_rate * w / np.pi, 20 * np.log10(np.abs(h)), 'b', linewidth=0.5)
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Gain (dB)')
        plt.grid()
        plt.title('Frequency Response of the FIR Low-pass Filter')
        plt.show()

    return filtered_data, fir_coefficients

# Based of the Tucker and Pitt
def double_integration_frequency_domain(one_sided_fft, frequencies, f1=0.04, f2=0.06, fc=2):
    """
    Perform double integration in the frequency domain with a high-pass filter.
    
    Parameters:
    one_sided_fft (array): The one-sided FFT of the signal.
    frequencies (array): The corresponding frequencies.
    f1 (float): Lower corner frequency for half cosine taper.
    f2 (float): Upper frequency for half cosine taper.
    fc (float): Cut-off frequency.
    
    Returns:
    displacement (array): The displacement signal obtained after double integration.
    filtered_fft (array): The filtered FFT of the signal.
    """
    
    N = len(frequencies)
    
    # Fourier filter
    Rf = np.zeros(N)
    
    for i in range(N):
        if frequencies[i] < f1:
            Rf[i] = 0
        elif f1 <= frequencies[i] < f2:
            Rf[i] = (1/2) * (1 - np.cos(np.pi * (frequencies[i] - f1) / (f2 - f1))) * (-1 / (2 * np.pi * frequencies[i])**2)
        elif f2 <= frequencies[i] < fc:
            Rf[i] = -1 / (2 * np.pi * frequencies[i])**2
    
    # Apply the filter in the frequency domain
    filtered_fft = np.abs(one_sided_fft * Rf)
    
    # Inverse Fourier transform to get the displacement signal
    displacement = np.fft.ifft(filtered_fft)
    
    return filtered_fft

# Define the model function for curve fitting (e.g., a power-law model)
def power_law(f, a, b):
    return a / (f**b)

def process_wave_data(verticalVelocity, sampleRate, duration, lowCut, highCut, plotAll=True, plotPSD = True, scaling='density'):
    """
    Process wave data to compute and plot the vertical velocity and displacement spectra using custom method.
    
    Parameters:
    verticalVelocity (array): The vertical velocity signal.
    sampleRate (float): The sampling rate of the signal.
    duration (float): The duration of the signal.
    lowCut (float): The low cut-off frequency for the FIR bandpass filter.
    highCut (float): The high cut-off frequency for the FIR bandpass filter.
    plotAll (bool): Whether to plot all intermediate results. Default is True.
    scaling (str): The scaling method for the PSD estimate. Default is 'density'.
    """
    
    # Define segment length, overlap and signal
    nperseg = int(1024 / 4)
    noverlap = nperseg // 2  # 50% Overlap
    signal = verticalVelocity

    time = np.linspace(0, duration, len(signal))

    # Plot the sampled signal
    if plotAll:
        plt.figure(figsize=(8, 4))
        plt.plot(time, signal, linewidth=0.5)
        plt.xlabel(r'Time $[s]$', size= 12)
        plt.ylabel(r'Vertical Velocity $[m/s]$', size=12)
        plt.xlim([100,356])
        #plt.ylim(bottom = 0)
        #plt.grid()
        plt.show()

    # Apply FIR bandpass filter to the signal
    signal, fir_coefficients = apply_fir_bandpass_filter(signal, sampleRate, lowCut, highCut)

    # Plot filtered signal
    if plotAll:
        plt.figure(figsize=(8, 4))
        plt.plot(time, signal, linewidth=0.5)
        plt.xlabel(r'Time $[s]$', size=12)
        plt.ylabel(r'Vertical Velocity $[m/s]$', size=12)
        plt.xlim([100, 356])
        #plt.ylim(bottom = 0)
        #plt.grid()
        plt.show()

    # Remove the first 100 samples
    num_samples_to_remove = 100
    signal = signal[num_samples_to_remove:]

    # Adjust the time array accordingly
    time = np.linspace(0, duration, len(verticalVelocity))
    time = time[num_samples_to_remove:]

    # Calculate the number of segments
    step = nperseg - noverlap
    shape = (signal.size - noverlap) // step
    segments = np.lib.stride_tricks.sliding_window_view(signal, nperseg)[::step]

    # Apply Hanning window to each segment
    window = np.hanning(nperseg)
    windowed_segments = segments * window

    if plotAll:
        # Plot the first windowed segment
        plt.figure(figsize=(8, 4))
        plt.plot(windowed_segments[1], linewidth=0.5)
        plt.xlabel(r'Time $[s]$', size=12)
        plt.ylabel(r'Windowed Vertical Velocity $[m/s]$', size=12)
        plt.xlim(left = 0)
        #plt.ylim(bottom = 0)
        #plt.grid()
        plt.show()

    # Compute the corresponding frequencies
    frequencies = np.fft.rfftfreq(nperseg, d=1/sampleRate)

    # Compute FFT of each segment
    fft_segments = np.fft.rfft(windowed_segments, axis=1) / (frequencies.size)  # Divide by length since it is a DFT
    fft_segments[:, 0] /= 2  # Divide the DC component by 2
    fft_segments[:, -1] /= 2  # Divide the Nyquist component by 2

    # Scale dependent on if you want spectrum or PSD (PSD preserves area under curve)
    if scaling == 'density':
        scale = (1.633**2) / (sampleRate / (frequencies.size))  # sample rate/array length = frequency resolution
    elif scaling == 'spectrum':
        scale = 1 / (window.sum()**2)

    # Compute periodogram for each segment and correct for energy loss due to window function
    periodograms = ((np.abs(fft_segments)) ** 2) * scale

    if plotAll:
        # Plot each periodogram and save the figures
        plt.figure(figsize=(8, 4))
        start_time = np.datetime64('2024-10-13 14:38:37') + np.timedelta64(100, 's')
        for i in range(len(periodograms)):
            segment_start_time = start_time + np.timedelta64(i * 128, 's')
            segment_end_time = segment_start_time + np.timedelta64(256, 's')
            plt.plot(frequencies, np.abs(periodograms[i]), linewidth=0.5, label=f'Segment {i+1}: {segment_start_time} to {segment_end_time}'.replace('T', ' '))
        plt.xlabel(r'Frequency [Hz]', size=12)
        plt.ylabel(r'$S_{v}(f)$ $[(m/s)^2/Hz]$', size=12)
        plt.xlim(left = 0)
        plt.ylim(bottom = 0)
        plt.show()

    # Average the periodograms
    velocitySpectrum = periodograms.mean(axis=0)

    # Integrate to get displacement
    displacementSpectrum = np.zeros_like(velocitySpectrum)

    displacementSpectrum = double_integration_frequency_domain(velocitySpectrum, frequencies, f1=0.04, f2=0.06, fc=1)

    #displacementSpectrum = detrend_spectrum(frequencies, displacementSpectrum, cutoff = 0.03)

    if plotAll:
        # Plot the velocity spectrum
        plt.figure(figsize=(8, 4))
        plt.plot(frequencies, velocitySpectrum, linewidth=0.5)
        plt.xlabel(r'Frequency $[Hz]$', size=12)
        plt.ylabel(r'$S_{v}(f)$ $[(m/s)^2/Hz]$', size=12)
        plt.xlim(left = 0)
        plt.ylim(bottom = 0)
        plt.show()


    df = 2*7  # Degrees of freedom for the confidence intervals
    lower, upper = calculate_psd_confidence_intervals(displacementSpectrum, df, confidence_level=0.90)

    if plotPSD:
        # Plot the elevation spectrum
        plt.figure(figsize=(8, 6))
        plt.plot(frequencies, displacementSpectrum, label = r'$S(f)$', linewidth=0.5)
        plt.plot(frequencies, lower, 'r', label='Lower Estimate', linewidth=0.5)
        plt.plot(frequencies, upper, 'g', label='Upper Estimate', linewidth=0.5)

        plt.xlabel(r'Frequency [Hz]', size=12)
        plt.xlim([0, 0.5])
        plt.ylim(bottom = 0)
        plt.ylabel(r'S(f) $[m^2/Hz]$', size=12)
        plt.legend()
        #plt.grid()
        plt.show()
    
    if plotAll:
        # Create a figure and a set of subplots
        fig, ax1 = plt.subplots(figsize=(8, 6))

        # Plot the displacement spectrum on the primary y-axis
        ax1.plot(frequencies, displacementSpectrum, label='Elevation PSD', linewidth=0.5, color='b')
        ax1.set_xlabel(r'Frequency [Hz]', size=12)
        ax1.set_ylabel(r'Amplitude $[m^2/Hz]$', size=12, color='b')
        ax1.tick_params(axis='y', labelcolor='b')

        # Create a secondary y-axis
        ax2 = ax1.twinx()
        ax2.plot(frequencies, velocitySpectrum, label='Velocity PSD', linewidth=0.5, color='r')
        ax2.set_ylabel(r'Amplitude $[(m/s)^2/Hz]$', size=12, color='r')
        ax2.tick_params(axis='y', labelcolor='r')

        # Add legends
        ax1.legend(loc='upper left')
        ax2.legend(loc='upper right')

        # Show the plot
        plt.show()


    """--------------------------------------------------------------------------------------------------------------
        Calculate the significant wave height, mean wave period, and average up-crossing period between waves
    --------------------------------------------------------------------------------------------------------------"""

    # Integrate using the trapezoidal rule
    m0 = np.trapz(displacementSpectrum, frequencies)
    m1 = np.trapz(frequencies * displacementSpectrum, frequencies)
    m2 = np.trapz((frequencies**2) * displacementSpectrum, frequencies)

    # Calculate the significant wave height (Hs)
    significant_wave_height = 4 * np.sqrt(m0)

    # Calculate the upper and lower estimates for significant wave height based on the confidence interval
    significant_wave_height_lower = 4 * np.sqrt(np.trapz(lower, frequencies))
    significant_wave_height_upper = 4 * np.sqrt(np.trapz(upper, frequencies))

    # Print the upper and lower estimates for significant wave height
    print(f"Lower Estimate of Significant Wave Height (Hs): {significant_wave_height_lower} meters")
    print(f"Upper Estimate of Significant Wave Height (Hs): {significant_wave_height_upper} meters")

    # Calculate the mean wave period
    mean_wave_period = m0 / m1

    # Calculate the average up-crossing period between waves
    average_zero_crossing_period = np.sqrt(m0 / m2)

    # Print the significant wave height
    print(f"Significant Wave Height (Hm0): {significant_wave_height} meters")

    # Print the mean wave period
    print(f"Mean Wave Period (Tm01): {mean_wave_period} seconds")

    # Print the average up-crossing period between waves
    print(f"Average up-crossing period between waves (T2): {average_zero_crossing_period} seconds")

    # Find the index of the maximum value in Pxx
    peak_index = np.argmax(displacementSpectrum)

    # Find the corresponding frequency
    peak_frequency = frequencies[peak_index]

    # Print the peak frequency
    print(f"Tp = {1/peak_frequency} seconds")    

def main():
    # Define the directory path and file names
    directory_path = '.\WD_2_GNSS\\WS0'
    file_names = ['W1.txt', 'W2.txt', 'W3.txt', 'W4.txt'] # 281 x 4 = 1124

    # Initialize an empty list to store the concatenated data
    velocity_data = []
    time_data = []

    # Loop through each file and read the last column of data
    for file_name in file_names:
        file_path = os.path.join(directory_path, file_name)
        with open(file_path, 'r') as file:
            for line in file:
                # Split the line by comma and extract the last part
                parts = line.strip().split(',')
                if len(parts) > 1:
                    time = parts[0].strip()
                    value = float(parts[-1].strip()) / 1000
                    time_data.append((time))
                    velocity_data.append((value))
                    time_data

    # Print the start and end time
    if time_data:
        start_time = time_data[0]
        end_time = time_data[-1]
        print(f"Start time: {start_time}")
        print(f"End time: {end_time}")

    # Concatenate the data from all files
    velocity_data = np.array(velocity_data)

    # Label the data sequentially
    labels = np.arange(1, len(velocity_data) + 1)

    # Combine the labels and data
    labeled_data = np.column_stack((labels, velocity_data))

    # Process the labeled data using the process_wave_data function
    F_SAMPLE = 1.0
    duration = 1124.0
    process_wave_data(labeled_data[:, 1], F_SAMPLE, duration, 0.03, 0.4, plotAll = False ,plotPSD= True, scaling='density')

if __name__ == "__main__":
    main()