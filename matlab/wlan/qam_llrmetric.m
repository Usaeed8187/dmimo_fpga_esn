function data = qam_llrmetric(rsyms, modtype, csi)
% Description
%   Hard decision demapping of QAM symbols
%
% Inputs:
%   rsyms     received QAM symbols (column vector)
%   modtype   Modulation type (BPSK/QPSK/16QAM)
%   csi       channel state information
% Output:
%   data	  Output bit stream (column vector)
%

narginchk(2,3)

rsyms = reshape(rsyms, [], 1);
if nargin >= 3
    csi = reshape(csi, [], 1);
end

switch modtype
    case 1
        data = 2.0 * real(rsyms);
    case 2
        metric = 2.0 * [real(rsyms), imag(rsyms)];
        data = reshape(metric.', [], 1);
    case 4
        data = zeros(4, numel(rsyms));
        if nargin == 2  % normalized (no CSI)
            data(4,:) = abs(imag(rsyms)) <= 2/sqrt(10);
            data(2,:) = abs(real(rsyms)) <= 2/sqrt(10);
        else  % using CSI
            data(4,:) = abs(imag(rsyms)) <= 2/sqrt(10)*csi;
            data(2,:) = abs(real(rsyms)) <= 2/sqrt(10)*csi;
        end
        data(3,:) = imag(rsyms) >= 0;
        data(1,:) = real(rsyms) >= 0;
        data = data(:);
    case 6
        data = zeros(6, numel(rsyms));
        if nargin == 2  % normalized (no CSI)
            data(6,:) = abs(imag(rsyms)) >= 2/sqrt(42) & abs(imag(rsyms)) <= 6/sqrt(42);
            data(5,:) = abs(imag(rsyms)) <= 4/sqrt(42);
            data(3,:) = abs(real(rsyms)) >= 2/sqrt(42) & abs(real(rsyms)) <= 6/sqrt(42);
            data(2,:) = abs(real(rsyms)) <= 4/sqrt(42);
        else  % using CSI
            data(6,:) = abs(imag(rsyms)) >= 2/sqrt(42)*csi & abs(imag(rsyms)) <= 6/sqrt(42)*csi;
            data(5,:) = abs(imag(rsyms)) <= 4/sqrt(42)*csi;
            data(3,:) = abs(real(rsyms)) >= 2/sqrt(42)*csi & abs(real(rsyms)) <= 6/sqrt(42)*csi;
            data(2,:) = abs(real(rsyms)) <= 4/sqrt(42)*csi;
        end
        data(4,:) = imag(rsyms) >= 0;
        data(1,:) = real(rsyms) >= 0;
        data = data(:);
    otherwise
        error('Unsupported modulation type')
end

