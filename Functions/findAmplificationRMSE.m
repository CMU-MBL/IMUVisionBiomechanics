function error = findAmplificationRMSE(z,signal,randNoise,desiredRMSE)
error = ( rms(signal - (signal + z*randNoise)) - desiredRMSE )^2;
end

