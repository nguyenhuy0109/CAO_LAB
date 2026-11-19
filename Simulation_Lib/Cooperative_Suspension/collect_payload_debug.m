function matFile = collect_payload_debug(modelFile, stopTime)
% COLLECT_PAYLOAD_DEBUG Run the USER'S ORIGINAL model and export plain data.
%
% FIRST run your ORIGINAL initialization script in the MATLAB base workspace.
% Then:
%   matFile = collect_payload_debug('Pay_load_fixed', 4);
% Or select your ORIGINAL working .slx file:
%   matFile = collect_payload_debug;
%
% Does not call init_payload_review or edit controller equations/gains/solver.
% Does not save_system or overwrite your model. Logging settings are restored.
% Only the known Draw_UAV_Load visualization is disabled for this diagnostic run.
% Stop time defaults to 4 seconds. Signal logging is not decimated.
% Send the resulting payload_debug_*.mat file back for investigation.
% Runtime errors are captured when possible; compile errors may have no samples.
% Intended for MATLAB/Simulink R2025a (original model version).
% This script has not been executed in MATLAB by its author.
%
% Official API references:
% https://www.mathworks.com/help/simulink/slref/sim.html
% https://www.mathworks.com/help/simulink/slref/signalproperties.html

if nargin<1 || isempty(modelFile)
    [fn,fd]=uigetfile({'*.slx;*.mdl','Original Simulink model'}, ...
        'Select YOUR ORIGINAL model (not Pay_load_reviewed)');
    if isequal(fn,0), matFile=''; return; end
    modelFile=fullfile(fd,fn);
end
if nargin<2 || isempty(stopTime), stopTime=4; end
validateattributes(stopTime,{'numeric'},{'scalar','real','finite','positive'});
modelFile=char(modelFile);
stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
folder=fullfile(pwd,'payload_debug_results');
[ok,msg]=mkdir(folder);
if ~ok, error('Cannot create output folder: %s',msg); end
matFile=fullfile(folder,['payload_debug_' stamp '.mat']);
debug=struct;
debug.format_version=1;
debug.created_local=char(datetime('now'));
debug.requested_model=modelFile;
debug.requested_stop_time=stopTime;
debug.matlab_version=version;
debug.toolbox_versions=ver;
debug.signals=struct;
debug.missing_signals={};
debug.notes={};
debug.error='';
debug.status='preparing';
debug.sources=struct('block',{},'code',{});

% A downloaded basename containing parentheses is not a MATLAB model name.
% If necessary, copy the original bytes to a valid basename, without XML edits.
[~,basename,ext]=fileparts(modelFile);
if ~isempty(ext) && ~isvarname(basename) && exist(modelFile,'file')==2
    validCopy=fullfile(folder,['payload_debug_source_' stamp ext]);
    copyfile(modelFile,validCopy);
    debug.notes{end+1}=['Original bytes copied to a valid model filename: ' validCopy];
    modelFile=validCopy;
end
try
    h=load_system(modelFile);
    mdl=get_param(h(1),'Name');
    debug.model=mdl;
    debug.model_file=get_param(mdl,'FileName');
    if ~strcmp(get_param(mdl,'SimulationStatus'),'stopped')
        error('Stop the existing simulation before running this collector.');
    end
catch ME
    debug.error=getReport(ME,'extended','hyperlinks','off');
    debug.status='load_failed';
    save(matFile,'debug','-v7');
    fprintf(2,'Could not load model. Error saved to:\n%s\n%s\n',matFile,debug.error);
    return;
end

% Capture only model-related parameters, not the entire personal workspace.
debug.config=capture_config(mdl);
debug.parameters_before=capture_parameters(mdl);
debug.sources=capture_sources(mdl);
debug.block_parameters=capture_blocks(mdl);

% Requests refer to actual model ports. For an input, log its connected source.
% This also works if a Derivative was renamed or replaced in the user's model.
R=cell(0,5); % alias, relative block path, input/output, port, essential
add('p_all','Mux','out',1,true);
add('v_all','Mux2','out',1,false);
add('xL','Integrator','out',1,true);
add('fCable_world','MATLAB Function3','out',2,false);
C='Subsystem4/PayLoad Controller/';
add('MdL',[C 'Equation (27)'],'out',1,true);
add('muL',[C 'Equation (27)'],'out',2,true);
for k=1:7
    a={'GammaL','z3L','sq_used','z4L','rhoJL','zetaL','delta2L'};
    add(a{k},[C 'Equation (27)'],'in',k,true);
end
add('sq_raw',[C 'Equation (77)'],'out',1,true);
for k=1:5
    a={'angleL_dot_for_Xi','PsiL','PsiL_dot','alpha_angleL_dot','alpha_angleL'};
    add(a{k},[C 'Equation (77)'],'in',k,false);
end
add('angleL',[C 'Equation (25)'],'in',1,false);
add('angleL_desired',[C 'Equation (25)'],'in',2,false);
add('FdL',[C 'Equation (21)'],'out',1,false);
add('alpha_pL_dot',[C 'Equation (21)'],'in',4,false);
add('z2L',[C 'Equation (21)'],'in',5,false);
add('delta1L',[C 'Equation (21)'],'in',6,false);
add('alpha_pL',[C 'Equation (18)'],'out',4,false);
add('eta_eL',[C 'Equation (18)'],'out',1,false);
add('pdL',[C 'Equation (18)'],'in',2,false);
add('pdL_dot',[C 'Equation (18)'],'in',3,false);
for i=1:4
    PC=sprintf('UAV%d Controller/Subsystem1/Position Controller/',i);
    VC=sprintf('UAV%d Controller/Subsystem1/Subsystem/',i);
    AC=sprintf('UAV%d Controller/Subsystem/',i);
    add(sprintf('alpha_%d',i),[PC 'Equation (38)'],'out',1,false);
    aliases={'Ei','eta_eij','deij','deta_eij','dei','eta_ei','deta_ei'};
    ports=[5 6 12 13 2 3 4];
    for j=1:numel(ports)
        add(sprintf('%s_%d',aliases{j},i),[PC 'Equation (38)'],'in',ports(j),j<=4);
    end
    add(sprintf('dij_%d',i),[PC 'MATLAB Function'],'in',1,false);
    add(sprintf('vij_%d',i),[PC 'MATLAB Function'],'in',3,false);
    add(sprintf('pd_%d',i),[PC 'Add'],'in',2,false);
    add(sprintf('pd_dot_%d',i),[PC 'Equation (38)'],'in',7,false);
    add(sprintf('alpha_dot_%d',i),[VC 'MATLAB Function'],'in',4,false);
    add(sprintf('delta_%d',i),[VC 'MATLAB Function'],'in',7,false);
    add(sprintf('rho_m_%d',i),[VC 'MATLAB Function'],'in',8,false);
    add(sprintf('ui_%d',i),[VC 'MATLAB Function'],'out',1,true);
    add(sprintf('z2_%d',i),[VC 'MATLAB Function'],'out',2,false);
    add(sprintf('ubar_%d',i),[VC 'MATLAB Function'],'out',3,false);
    add(sprintf('rho_m_dot_%d',i),[VC 'MATLAB Function'],'out',5,false);
    add(sprintf('thrust_raw_%d',i),[VC 'MATLAB Function1'],'out',1,false);
    add(sprintf('attitude_ref_%d',i),[VC 'MATLAB Function1'],'out',2,false);
    add(sprintf('valid_%d',i),[VC 'MATLAB Function1'],'out',3,false);
    add(sprintf('thrust_%d',i),[VC 'Saturation'],'out',1,true);
    add(sprintf('angle_%d',i),[AC 'MATLAB Function'],'in',1,false);
    add(sprintf('omega_%d',i),[AC 'MATLAB Function'],'in',2,false);
    add(sprintf('z4_%d',i),[AC 'MATLAB Function'],'out',2,false);
    add(sprintf('tau_%d',i),[AC 'MATLAB Function2'],'out',1,false);
    add(sprintf('Xi_%d',i),[AC 'MATLAB Function2'],'in',5,false);
end

ports=struct('handle',{},'key',{},'aliases',{},'block',{},'port',{},'before',{});
restoreGuard=onCleanup(@restore_logging); %#ok<NASGU>
for k=1:size(R,1)
    try
        target=[mdl '/' R{k,2}];ph=get_param(target,'PortHandles');
        if strcmp(R{k,3},'out')
            portH=ph.Outport(R{k,4});
        else
            lineH=get_param(ph.Inport(R{k,4}),'Line');
            if lineH<0,error('Input is not connected.');end
            portH=get_param(lineH,'SrcPortHandle');
        end
        if isempty(portH) || portH<0,error('Source port not found.');end
        existing=find([ports.handle]==portH,1);
        if isempty(existing)
            n=numel(ports)+1;
            ports(n).handle=portH;
            ports(n).key=sprintf('payload_probe_%03d',n);
            ports(n).aliases={R{k,1}};
            ports(n).block=get_param(portH,'Parent');
            ports(n).port=get_param(portH,'PortNumber');
            props={'DataLogging','DataLoggingNameMode','DataLoggingName', ...
                'DataLoggingDecimateData','DataLoggingLimitDataPoints'};
            ports(n).before=struct;
            for j=1:numel(props)
                try,ports(n).before.(props{j})=get_param(portH,props{j});catch,end
            end
            set_param(portH,'DataLogging','on','DataLoggingNameMode','Custom', ...
                'DataLoggingName',ports(n).key);
            if isfield(ports(n).before,'DataLoggingDecimateData')
                set_param(portH,'DataLoggingDecimateData','off');
            end
            if isfield(ports(n).before,'DataLoggingLimitDataPoints')
                set_param(portH,'DataLoggingLimitDataPoints','off');
            end
        else
            ports(existing).aliases{end+1}=R{k,1};
        end
    catch ME
        debug.missing_signals{end+1}=sprintf('%s | %s | %s',R{k,1},R{k,2},ME.message);
        if R{k,5},fprintf(2,'Missing important signal: %s\n',R{k,1});end
    end
end
debug.logging_map=rmfield(ports,{'handle','before'});
debug.status='configured';
save(matFile,'debug','-v7'); % Checkpoint even if simulation cannot start.
fprintf('Configured %d unique signal ports. Original solver: %s, step: %s\n', ...
    numel(ports),debug.config.Solver,debug.config.FixedStep);
fprintf('Running original model to %g s...\n',stopTime);

out=[];
try
    simIn=Simulink.SimulationInput(mdl);
    simIn=simIn.setModelParameter('StopTime',num2str(stopTime,17), ...
        'ReturnWorkspaceOutputs','on','SignalLogging','on', ...
        'SignalLoggingName','payload_debug_logs','DatasetSignalFormat','timeseries', ...
        'SaveTime','on','TimeSaveName','payload_debug_time','CaptureErrors','on');
    % Do not alter forces/dynamics. Disable only the known display S-function.
    bs=find_system(mdl,'LookUnderMasks','all','FollowLinks','on','BlockType','M-S-Function');
    for j=1:numel(bs)
        if strcmp(get_param(bs{j},'FunctionName'),'Draw_UAV_Load')
            simIn=simIn.setBlockParameter(bs{j},'Commented','on');
            debug.notes{end+1}=['Visualization disabled for this run: ' bs{j}];
        end
    end
    out=sim(simIn);
    debug.error=out.ErrorMessage;
    if isempty(debug.error),debug.status='completed';else,debug.status='simulation_error';end
    try,debug.execution_info=evalc('disp(out.SimulationMetadata.ExecutionInfo)');catch,end
    try
        e=out.SimulationMetadata.ExecutionInfo.ErrorDiagnostic;
        if ~isempty(e)
            debug.error_time=e.SimulationTime;
            debug.error_phase=char(e.SimulationPhase);
        end
    catch
    end
catch ME
    debug.status='exception';debug.error=getReport(ME,'extended','hyperlinks','off');
end
% Include values after InitFcn if it reinitializes model parameters.
debug.parameters_after=capture_parameters(mdl);
if ~isempty(out)
    try
        ds=out.get('payload_debug_logs');
        for j=1:numel(ports)
            try
                el=ds.getElement(ports(j).key); s=plain_timeseries(el.Values);
                s.block=ports(j).block;s.port=ports(j).port;
                for a=1:numel(ports(j).aliases)
                    debug.signals.(ports(j).aliases{a})=s;
                end
            catch ME
                debug.notes{end+1}=sprintf('Export failed for %s: %s',ports(j).key,ME.message);
            end
        end
    catch ME
        debug.notes{end+1}=['No exported dataset: ' ME.message];
    end
end
debug.exported_signal_count=numel(fieldnames(debug.signals));
try,debug.derived=derive_metrics(debug);catch ME,debug.notes{end+1}=['Derived metrics: ' ME.message];end
save(matFile,'debug','-v7'); % Plain numeric structs: readable without Simulink.
fprintf('\nSaved %d signal aliases. Status: %s\n%s\n', ...
    debug.exported_signal_count,debug.status,matFile);
if ~isempty(debug.error),fprintf(2,'Simulation error:\n%s\n',debug.error);end
if debug.exported_signal_count==0
    fprintf(2,'No time samples. The MAT file still contains error/configuration/parameter information.\n');
end
try,quick_plot(debug);catch ME,fprintf('Plot skipped: %s\n',ME.message);end
fprintf('Send this payload_debug_*.mat file for analysis.\n');

    function add(alias,path,kind,index,essential)
        R(end+1,:)={alias,path,kind,index,essential};
    end
    function restore_logging
        for a=1:numel(ports)
            fields=fieldnames(ports(a).before);
            for b=1:numel(fields)
                try,set_param(ports(a).handle,fields{b},ports(a).before.(fields{b}));catch,end
            end
        end
        % No save_system: the .slx on disk is never overwritten.
    end
end

function c=capture_config(mdl)
c=struct;
keys={'Solver','SolverType','FixedStep','MaxStep','RelTol','AbsTol','StartTime', ...
      'StopTime','SimulationMode','InitFcn','StartFcn','StopFcn','PreLoadFcn'};
for k=1:numel(keys)
    try,c.(keys{k})=get_param(mdl,keys{k});catch,c.(keys{k})='unavailable';end
end
end

function p=capture_parameters(mdl)
p=struct('base',struct,'model',struct);
keys={'P','m','mL','Iv','JL','joint_matrix','xL_init','pL_init','angleL_init', ...
      'm1','m2','m3','m4','r1','r2','r3','r4','p1_init','p2_init','p3_init','p4_init', ...
      'angle1_init','angle2_init','angle3_init','angle4_init'};
mw=get_param(mdl,'ModelWorkspace');
for k=1:numel(keys)
    key=keys{k};
    try,v=evalin('base',key);if plain_value(v),p.base.(key)=v;end,catch,end
    try,v=getVariable(mw,key);if plain_value(v),p.model.(key)=v;end,catch,end
end
end
function ok=plain_value(v)
ok=isnumeric(v)||islogical(v)||ischar(v)||isstruct(v)||iscell(v)||isstring(v);
if isstruct(v)
    fields=fieldnames(v);
    for i=1:numel(v)
        for j=1:numel(fields),ok=ok&&plain_value(v(i).(fields{j}));end
    end
elseif iscell(v)
    for i=1:numel(v),ok=ok&&plain_value(v{i});end
end
end

function s=capture_sources(mdl)
s=struct('block',{},'code',{});
try
    root=sfroot;charts=root.find('-isa','Stateflow.EMChart');
    for k=1:numel(charts)
        path=charts(k).Path;
        if startsWith(path,[mdl '/'])
            s(end+1)=struct('block',path,'code',charts(k).Script); %#ok<AGROW>
        end
    end
catch
end
end

function b=capture_blocks(mdl)
b=struct('path',{},'type',{},'parameters',{});
blocks=find_system(mdl,'LookUnderMasks','all','FollowLinks','on','Type','Block');
for k=1:numel(blocks)
    typ=get_param(blocks{k},'BlockType');
    if ~ismember(typ,{'Integrator','Saturate','Gain','Derivative'}),continue;end
    keys={'InitialCondition','LimitOutput','UpperSaturationLimit','LowerSaturationLimit', ...
          'UpperLimit','LowerLimit','Gain','SampleTime'};
    v=struct;
    for j=1:numel(keys),try,v.(keys{j})=get_param(blocks{k},keys{j});catch,end,end
    b(end+1)=struct('path',blocks{k},'type',typ,'parameters',v); %#ok<AGROW>
end
end

function s=plain_timeseries(ts)
if ~isa(ts,'timeseries'),error('Expected timeseries, received %s.',class(ts));end
s.time=double(ts.Time(:));raw=ts.Data;n=numel(s.time);
s.raw_size=size(raw);s.is_time_first=ts.IsTimeFirst;
if n==0,s.data=[];return;end
if ~(isnumeric(raw)||islogical(raw)),error('Non-numeric signal.');end
if ts.IsTimeFirst,s.data=reshape(double(raw),n,[]);else,s.data=reshape(double(raw),[],n).';end
end

function D=derive_metrics(debug)
D=struct;S=debug.signals;
D.barrier_formula_verified=false;
% Avoid silently applying paper bounds if the user changed their functions.
checked=0;
for k=1:numel(debug.sources)
    c=regexprep(debug.sources(k).code,'\s','');
    if contains(c,'function[d_eij,eta_eij,d_eta_d_deij]')
        if ~(contains(c,'Omega_Hij=(2-0.1)*exp(-0.1*t)+0.1;') && ...
             contains(c,'Omega_Wij=(1.8-0.1)*exp(-0.15*t)+0.1;'))
            checked=-100;break;
        end
        checked=checked+1;
    end
end
D.barrier_formula_verified=(checked==4);
D.note='Gap calculations only enabled when all four saved chart sources match the original barrier functions.';
D.pair_order=[2 3 4;1 3 4;1 2 4;1 2 3];
for i=1:4
    name=sprintf('deij_%d',i);
    if isfield(S,name) && D.barrier_formula_verified
        s=S.(name);t=s.time;
        gH=1.9*exp(-0.1*t)+0.1-s.data;
        gW=1.7*exp(-0.15*t)+0.1+s.data;
        D.(sprintf('gapH_%d',i))=struct('time',t,'data',gH);
        D.(sprintf('gapW_%d',i))=struct('time',t,'data',gW);
    end
end
end

function quick_plot(debug)
S=debug.signals;if isempty(fieldnames(S)),return;end
figure('Name','Original payload model diagnostic','Color','w');
subplot(3,2,1);hold on;
if isfield(debug,'derived') && debug.derived.barrier_formula_verified
    for i=1:4
        key=sprintf('gapH_%d',i);
        if isfield(debug.derived,key)
            s=debug.derived.(key);plot(s.time,min(s.data,[],2),'DisplayName',sprintf('UAV%d',i));
        end
    end
    ylabel('Minimum upper gap (m)');legend('show');
else
    plot_signal(S,'eta_eij_1');ylabel('eta_eij, UAV1');
end
grid on;
subplot(3,2,2);plot_signal(S,'sq_raw');ylabel('Xi_L raw');grid on;
subplot(3,2,3);plot_signal(S,'MdL');ylabel('M_dL');grid on;
subplot(3,2,4);hold on;
for name={'rhoJL','zetaL','delta2L'}
    key=name{1};if isfield(S,key),s=S.(key);plot(s.time,s.data,'DisplayName',key);end
end
legend('show');ylabel('Adaptive estimates');grid on;
subplot(3,2,5);hold on;
for i=1:4
    key=sprintf('thrust_%d',i);
    if isfield(S,key),s=S.(key);plot(s.time,s.data,'DisplayName',sprintf('UAV%d',i));end
end
legend('show');ylabel('Actual thrust (N)');grid on;
subplot(3,2,6);plot_signal(S,'deta_eij_1');ylabel('d eta / d error, UAV1');grid on;
end
function plot_signal(S,key)
if isfield(S,key),s=S.(key);plot(s.time,s.data);else,title(['Missing: ' key],'Interpreter','none');end
xlabel('Time (s)');
end
