function msfun_test(block)
setup(block);
function setup(block)

% Register number of ports
block.NumInputPorts  = 1;
block.NumOutputPorts = 1;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions        = 1;                                   % the inputs are demanded power and SOC
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;
block.InputPort(1).SamplingMode = 'Sample';

% Override output port properties
block.OutputPort(1).Dimensions       = 2;                                   % the outputs are APU power and battery power
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';
% Register parameters
block.NumDialogPrms     = 0;
block.SampleTimes = [-1 0];                                                  % the sampling time is 1s
block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Update', @Update);
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate); % Required

function DoPostPropSetup(block)
function InitializeConditions(block)
function Start(block)

function Outputs(block)
st=block.InputPort(1).Data(1);
step=st;
v_act_k=traci.vehicle.getSpeed('vehicle_0');%v(k), mps
a_max=0.7;%�����ٶ�
v0=traci.vehicle.getAllowedSpeed('vehicle_0');%��������
s0=2;%Jam distance
s1=0;%Jam Distance
T=1.6;%��ȫ��ͷʱ��
b=0.7;%�������ٶ�
[leader,s_alpha]=traci.vehicle.getLeader('vehicle_0');%ʵ�ʳ����
delta=4;%���ٶ�ָ��
%����һ�����̵Ƶľ���
TLS=traci.vehicle.getNextTLS('vehicle_0');
if length(TLS)>0%�к��̵Ƶľ������
    nextTLS=TLS(1);
    gap2nextTLS=cell2mat(nextTLS{1}(3));
    state_of_nextTLS=cell2mat(nextTLS{1}(4));
else
    gap2nextTLS=1e6;
    state_of_nextTLS='nan';
end
if v_act_k<0%����ǰ
    a_pre=0;
else%������
    if gap2nextTLS>1e3%�����Զ
        if length(leader)>0%����ǰ��
            v_leader=traci.vehicle.getSpeed(leader);
            delta_v=v_act_k-v_leader;%�󳶺�ǰ�����ٲ�
            s_star=s0+s1*sqrt(v_act_k/v0)+T*v_act_k+v_act_k*delta_v/2/sqrt(a_max*b);%��С��ȫ�����
            s_alpha=s_alpha+traci.vehicle.getMinGap('vehicle_0');
            a_pre=a_max*(1-(v_act_k/v0)^delta-(s_star/s_alpha)^2);%Ԥ����ٶ�
        else %������ǰ��
            a_pre=a_max*(1-(v_act_k/v0)^delta);
        end
    else%����ƽ�
        if length(leader)>0%����ǰ��
            if state_of_nextTLS=='r'||state_of_nextTLS=='y'%���
                v_leader=traci.vehicle.getSpeed(leader);
                if (s_alpha+traci.vehicle.getMinGap('vehicle_0'))<=gap2nextTLS%��ǰ������
                    delta_v=v_act_k-v_leader;%�󳵺�ǰ�����ٲ�
                else%��Ƹ���
                    delta_v=v_act_k-0;
                end
                s_star=s0+s1*sqrt(v_act_k/v0)+T*v_act_k+v_act_k*delta_v/2/sqrt(a_max*b);%��С��ȫ�����
                s_alpha=min(s_alpha+traci.vehicle.getMinGap('vehicle_0'),gap2nextTLS);
                a_pre=a_max*(1-(v_act_k/v0)^delta-(s_star/s_alpha)^2);%Ԥ����ٶ�
            else %�̵�
                v_leader=traci.vehicle.getSpeed(leader);
                delta_v=v_act_k-v_leader;%�󳵺�ǰ�����ٲ�
                s_star=s0+s1*sqrt(v_act_k/v0)+T*v_act_k+v_act_k*delta_v/2/sqrt(a_max*b);%��С��ȫ�����
                s_alpha=s_alpha+traci.vehicle.getMinGap('vehicle_0');
                a_pre=a_max*(1-(v_act_k/v0)^delta-(s_star/s_alpha)^2);%Ԥ����ٶ�
            end
        else%������ǰ��
            if state_of_nextTLS=='r'||state_of_nextTLS=='y'%���
                delta_v=v_act_k;%�󳵺�ǰ�����ٲ�
                s_star=s0+s1*sqrt(v_act_k/v0)+T*v_act_k+v_act_k*delta_v/2/sqrt(a_max*b);%��С��ȫ�����
                s_alpha=gap2nextTLS;
                a_pre=a_max*(1-(v_act_k/v0)^delta-(s_star/s_alpha)^2);%Ԥ����ٶ�
            else%�̵�
                a_pre=a_max*(1-(v_act_k/v0)^delta);
            end
        end
    end
end
v_act_k(v_act_k<0)=0;
v_pre_kplus1=v_act_k+a_pre*0.01;
traci.simulation.step()
block.OutputPort(1).Data(1) = v_act_k;
v_pre_kplus1(v_pre_kplus1<0)=0;
block.OutputPort(1).Data(2) = v_pre_kplus1;

function Update(block)
function Derivatives(block)
function Terminate(block)