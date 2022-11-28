function error = findAmplificationSNR(x,signal,randNoise,desiredSNR)
error = ( ( (rms(signal)^2 / rms(x*randNoise)^2 ) ) - desiredSNR )^2;
end

