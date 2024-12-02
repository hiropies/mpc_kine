function filtered_wave = LPF(wave)

% サンプル周波数
Fs = 1/0.0002; % サンプリング周波数 (Hz)
% ナイキスト周波数
Fn = Fs/2; % ナイキスト周波数 (Hz)

% フィルタ設計
% パラメータ設定
order = 2; % フィルタ次数 (オーダー)
cutoffFreq = 5; % カットオフ周波数 (Hz)
% バターワースフィルタ設計
[b, a] = butter(order, cutoffFreq/Fn);
%disp(b);
%disp(a);

% フィルタ応答を表示
freqz(b, a, [], Fs);
subplot(2,1,1)

% 入力信号生成
x = wave;
disp(length(wave));

% フィルタ適用
filtered_wave = filter(b, a, x);