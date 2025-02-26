function data = qam_demapping(rsyms, modtype, csi)
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
        data = real(rsyms) >= 0;
    case 2
        metric = [-imag(rsyms), real(rsyms)];
        data = reshape(metric.', [], 1) > 0;
    case 4
        data = zeros(4, numel(rsyms));
        if nargin == 2  % normalized (no CSI)
            data(1,:) = abs(imag(rsyms)) <= 2/sqrt(10);
            data(3,:) = abs(real(rsyms)) <= 2/sqrt(10);
        else  % using CSI
            data(1,:) = abs(imag(rsyms)) <= 2/sqrt(10)*csi;
            data(3,:) = abs(real(rsyms)) <= 2/sqrt(10)*csi;
        end
        data(2,:) = imag(rsyms) < 0;
        data(4,:) = real(rsyms) >= 0;
        data = data(:);
    case 6
        data = zeros(6, numel(rsyms));
        if nargin == 2  % normalized (no CSI)
            data(1,:) = abs(imag(rsyms)) >= 2/sqrt(42) & abs(imag(rsyms)) <= 6/sqrt(42);
            data(2,:) = abs(imag(rsyms)) <= 4/sqrt(42);
            data(4,:) = abs(real(rsyms)) >= 2/sqrt(42) & abs(real(rsyms)) <= 6/sqrt(42);
            data(5,:) = abs(real(rsyms)) <= 4/sqrt(42);
        else  % using CSI
            data(1,:) = abs(imag(rsyms)) >= 2/sqrt(42)*csi & abs(imag(rsyms)) <= 6/sqrt(42)*csi;
            data(2,:) = abs(imag(rsyms)) <= 4/sqrt(42)*csi;
            data(4,:) = abs(real(rsyms)) >= 2/sqrt(42)*csi & abs(real(rsyms)) <= 6/sqrt(42)*csi;
            data(5,:) = abs(real(rsyms)) <= 4/sqrt(42)*csi;
        end
        data(3,:) = imag(rsyms) < 0;
        data(6,:) = real(rsyms) >= 0;
        data = data(:);
    case 8
        data = zeros(8, numel(rsyms));
        if nargin == 2 % normalized (no CSI)
            data(1,:) = abs(abs(abs(imag(rsyms)) - 8/sqrt(170))-4/sqrt(170)) <= 2/sqrt(170);
            data(2,:) = abs(abs(imag(rsyms)) - 8/sqrt(170)) <= 4/sqrt(170);
            data(3,:) = abs(imag(rsyms)) <= 8/sqrt(170);
            data(5,:) = abs(abs(abs(real(rsyms)) - 8/sqrt(170))-4/sqrt(170)) <= 2/sqrt(170);
            data(6,:) = abs(abs(real(rsyms)) - 8/sqrt(170)) <= 4/sqrt(170);
            data(7,:) = abs(real(rsyms)) <= 8/sqrt(170);
        end
        data(4,:) = imag(rsyms) <= 0;
        data(8,:) = real(rsyms) >= 0;
        data = data(:);
    otherwise
        error('Unsupported modulation type')
end

