% =========================================================================
% PROJETO UC INTELIGÊNCIA ARTIFICIAL - Engenharia de Controle e Automação
% SIMULAÇÃO DE CONTROLE ATIVO DE VIBRAÇÃO VIA LÓGICA FUZZY (VIGA ENGASTADA)
% =========================================================================
clear; clc; close all;

%% 1. PARAMETRIZAÇÃO E MATRIZES EM ESPAÇO DE ESTADOS (MODELO CONTÍNUO)
% Parâmetros dinâmicos baseados no 1º modo de vibração (10 Hz, 1% amortecimento)
A = [0 1; -3948 -1.26];
Bu = [0; 5.2];    % Matriz de entrada do Atuador de Controle
Bw = [0; 2.5];    % Matriz de entrada do Distúrbio (Motor)
Buw = [Bu, Bw];
C = [12.5 0; -7896 -2.52]; % Saídas: [1] Extensômetro (Base), [2] Acelerômetro (Ponta)
Du = [0; 10.4];
Dw = [0; 5.0];
Duw = [Du, Dw];

%% 2. DISCRETIZAÇÃO DO SISTEMA (Simulação de Controle Digital)
dt = 0.002; % Passo de amostragem de 2 ms (Frequência de amostragem de 500 Hz)

B_total = [Bu Bw];
D_total = [Du Dw];
sys_continuo = ss(A, B_total, C, D_total);
sys_discreto = c2d(sys_continuo, dt, 'zoh'); % Discretização por Zero-Order Hold

% Extração das matrizes discretas
Ad = sys_discreto.A;
Bd_u = sys_discreto.B(:,1);
Bd_w = sys_discreto.B(:,2);
Cd = sys_discreto.C;
Dd_u = sys_discreto.D(:,1);
Dd_w = sys_discreto.D(:,2);

%% 3. CRIAÇÃO PROGRAMÁTICA DO CONTROLADOR FUZZY (5 CONJUNTOS - FASE CORRIGIDA)
fis = newfis('ControleVibracao5Regras');

% Mapeamento exato de Índices: 1=NG, 2=NP, 3=Z, 4=PP, 5=PG

% Entrada 1: Deformação 
fis = addvar(fis, 'input', 'Deformacao', [-20 20]);
fis = addmf(fis, 'input', 1, 'NG', 'trimf', [-20 -20 -10]);
fis = addmf(fis, 'input', 1, 'NP', 'trimf', [-15 -6 0]);
fis = addmf(fis, 'input', 1, 'Z',  'trimf', [-2 0 2]);
fis = addmf(fis, 'input', 1, 'PP', 'trimf', [0 6 15]);
fis = addmf(fis, 'input', 1, 'PG', 'trimf', [10 20 20]);

% Entrada 2: Aceleração
fis = addvar(fis, 'input', 'Aceleracao', [-15000 15000]);
fis = addmf(fis, 'input', 2, 'NG', 'trimf', [-15000 -15000 -5000]);
fis = addmf(fis, 'input', 2, 'NP', 'trimf', [-8000 -3000 0]);
fis = addmf(fis, 'input', 2, 'Z',  'trimf', [-1000 0 1000]);
fis = addmf(fis, 'input', 2, 'PP', 'trimf', [0 3000 8000]);
fis = addmf(fis, 'input', 2, 'PG', 'trimf', [5000 15000 15000]);

% Saída: Controle
fis = addvar(fis, 'output', 'Controle', [-40 40]);
fis = addmf(fis, 'output', 1, 'NG', 'trimf', [-40 -40 -20]);
fis = addmf(fis, 'output', 1, 'NP', 'trimf', [-25 -12 0]);
fis = addmf(fis, 'output', 1, 'Z',  'trimf', [-1 0 1]);
fis = addmf(fis, 'output', 1, 'PP', 'trimf', [0 12 25]);
fis = addmf(fis, 'output', 1, 'PG', 'trimf', [20 40 40]);

% BASE DE REGRAS CRUCIAL: Inversão de Sinais para Realimentação Negativa
% Linha: [Entrada1_Deformacao  Entrada2_Aceleracao  Saida_Controle  Peso  Operador]
regras = [
    5 0 1 1 1; % Se Deformacao é PG (5) Então Controle é NG (1) -> ANTI-FASE
    4 0 2 1 1; % Se Deformacao é PP (4) Então Controle é NP (2) -> ANTI-FASE
    1 0 5 1 1; % Se Deformacao é NG (1) Então Controle é PG (5) -> ANTI-FASE
    2 0 4 1 1; % Se Deformacao é NP (2) Então Controle é PP (4) -> ANTI-FASE
    3 3 3 1 1  % Se ambas são Z (3) Então Controle é Z (3) -> Equilíbrio
];
fis = addrule(fis, regras);

%% 4. LOOP DE SIMULAÇÃO COMPUTACIONAL
t = 0:dt:5; % Vetor de tempo: 4 segundos de simulação
N = length(t);

% Geração do Distúrbio do Motor monitorado via Encoder
% Força o sistema na frequência de Ressonância exata da viga (~10 Hz -> W = 62.83 rad/s)
w = 1.8 * sin(62.83 * t); 

% --- Cenário 1: Malha Aberta (Sem Controle Fuzzy, u = 0) ---
x_ma = [0; 0];          % Vetor de estados inicial [deslocamento_modal; velocidade_modal]
y_ma = zeros(2, N);     % Armazena as saídas dos sensores
for k = 1:N
    % Leitura instantânea dos sensores teóricos
    y_ma(:, k) = Cd * x_ma + Dd_w * w(k);
    
    % Atualização do estado físico da viga (Apenas sob efeito do motor)
    x_ma = Ad * x_ma + Bd_w * w(k);
end

% --- Cenário 2: Malha Fechada ---
x_mf = [0; 0];          
y_mf = zeros(2, N);     
u = zeros(1, N);        
u_anterior = 0;         

for k = 1:N
    y_atual = Cd * x_mf + Dd_u * u_anterior + Dd_w * w(k);
    y_mf(:, k) = y_atual;
    
    u_fuzzy = evalfis([y_atual(1), y_atual(2)], fis);
    
    u(k) = -u_fuzzy;
    u_anterior = u_fuzzy; 
    
    x_mf = Ad * x_mf + Bd_u * u(k) + Bd_w * w(k);
end

%% 5. APRESENTAÇÃO E ANÁLISE GRÁFICA DOS CONJUNTOS

% 1. Gráfico para a Primeira Entrada (Sensor de Deformação)
figure('Name', 'Funções de Pertinência - Deformação');
plotmf(fis, 'input', 1);
set(findobj(gca, 'Type', 'line'), 'LineWidth', 2);
title('Conjuntos Fuzzy: Sensor de Deformação (\epsilon)');
xlabel('Universo de Discurso (Entrada 1)');
ylabel('Grau de Pertinência (\mu)');
grid on;

% 2. Gráfico para a Segunda Entrada (Acelerômetro)
figure('Name', 'Funções de Pertinência - Aceleração');
plotmf(fis, 'input', 2);
set(findobj(gca, 'Type', 'line'), 'LineWidth', 2);
title('Conjuntos Fuzzy: Sensor de Aceleração (mm/s²)');
xlabel('Universo de Discurso (Entrada 2)');
ylabel('Grau de Pertinência (\mu)');
grid on;

% 3. Gráfico para a Saída (Atuador de Controle)
figure('Name', 'Funções de Pertinência - Atuador');
plotmf(fis, 'output', 1);
set(findobj(gca, 'Type', 'line'), 'LineWidth', 2);
title('Conjuntos Fuzzy: Esforço de Controle do Atuador (u)');
xlabel('Universo de Discurso (Saída)');
ylabel('Grau de Pertinência (\mu)');
grid on;

%% 5. APRESENTAÇÃO E ANÁLISE GRÁFICA DOS RESULTADOS
figure('Name', 'Resultados do Controle Ativo Fuzzy', 'NumberTitle', 'off');

% Subplot 1: Sensor de Deformação (Ponte de Extensômetros)
subplot(2,1,1);
plot(t, y_ma(1,:), 'r', 'LineWidth', 1.2); hold on;
plot(t, y_mf(1,:), 'b', 'LineWidth', 1.5);
grid on;
title('Resposta do Sensor de Deformação (Ponte de Extensômetros na Base)');
xlabel('Tempo (s)');
ylabel('Tensão Equivalente / Deformação (\epsilon)');
legend('Malha Aberta (Sem Controle)', 'Malha Fechada (Controle Fuzzy)');

% Subplot 2: Sensor de Aceleração (Acelerômetro na Ponta)
subplot(2,1,2);
plot(t, y_ma(2,:), 'r', 'LineWidth', 1.2); hold on;
plot(t, y_mf(2,:), 'b', 'LineWidth', 1.5);
grid on;
title('Resposta do Acelerômetro (Extremidade Livre da Viga)');
xlabel('Tempo (s)');
ylabel('Aceleração (mm/s²)');
legend('Malha Aberta (Sem Controle)', 'Malha Fechada (Controle Fuzzy)');

% Cálculo da eficiência de mitigação em Regime Permanente (Último 1 segundo)
idx_regime = t > 3.0;
rms_ma = rms(y_ma(2, idx_regime));
rms_mf = rms(y_mf(2, idx_regime));
atenuacao_pct = (1 - (rms_mf / rms_ma)) * 100;

fprintf('====================================================\n');
fprintf('ANÁLISE DE DESEMPENHO DO CONTROLADOR FUZZY (SIMULAÇÃO)\n');
fprintf('====================================================\n');
fprintf('Aceleração RMS em Malha Aberta: %.2f mm/s²\n', rms_ma);
fprintf('Aceleração RMS em Malha Fechada: %.2f mm/s²\n', rms_mf);
fprintf('Atenuação percentual de vibração obtida: %.2f%%\n', atenuacao_pct);
fprintf('====================================================\n');
