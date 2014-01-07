unit MainUnit;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, ExtCtrls, StdCtrls, ComCtrls, Synaser,
  fileUtil, ClipBrd;

type

  { TMainForm }

  TMainForm = class(TForm)
    AvgDetFill: TPanel;
    BaudRate: TComboBox;
    floppy: TImage;
    comms: TImage;
    Label23: TLabel;
    Label24: TLabel;
    CaptureFileName: TLabel;
    info2: TLabel;
    NoResponse: TLabel;
    Info: TLabel;
    Label17: TLabel;
    Label18: TLabel;
    Label20: TLabel;
    Label8: TLabel;
    CommErr: TLabel;
    Panel25: TPanel;
    Panel28: TPanel;
    Run1: TButton;
    SaveNVM: TButton;
    Status: TLabel;
    Panel10: TPanel;
    Panel20: TPanel;
    log0: TRadioButton;
    Log2: TRadioButton;
    Log1: TRadioButton;
    SavePanel: TPanel;
    Panel27: TPanel;
    ResetSensor: TButton;
    Run4: TButton;
    Run3: TButton;
    SaveTemp: TButton;
    SetComms: TButton;
    Timer1: TTimer;
    Tollerance: TEdit;
    RejOnTime: TEdit;
    SampleRate: TEdit;
    Neck: TEdit;
    Cap: TEdit;
    Algorithm: TEdit;
    TMin: TEdit;
    Label10: TLabel;
    Label11: TLabel;
    Label12: TLabel;
    Label13: TLabel;
    Label14: TLabel;
    Label15: TLabel;
    Label16: TLabel;
    Label19: TLabel;
    Label21: TLabel;
    Label22: TLabel;
    Label7: TLabel;
    Label9: TLabel;
    p1: TPaintBox;
    Panel11: TPanel;
    Panel12: TPanel;
    Panel13: TPanel;
    Panel14: TPanel;
    Panel15: TPanel;
    Panel16: TPanel;
    Panel17: TPanel;
    Panel18: TPanel;
    Panel19: TPanel;
    Panel21: TPanel;
    BottleCount: TPanel;
    Panel24: TPanel;
    Panel26: TPanel;
    RejectCount: TPanel;
    Exposure: TPanel;
    Panel8: TPanel;
    Panel9: TPanel;
    SetWorkingDir: TButton;
    COMPort: TComboBox;
    DataDir: TEdit;
    Label2: TLabel;
    Label3: TLabel;
    Label4: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    log: TMemo;
    OffLinePanel: TPanel;
    PageControl1: TPageControl;
    Panel2: TPanel;
    Panel4: TPanel;
    Panel5: TPanel;
    Panel6: TPanel;
    Panel7: TPanel;
    Label1: TLabel;
    Panel22: TPanel;
    Panel1: TPanel;
    Panel3: TPanel;
    TabSheet1: TTabSheet;
    TabSheet2: TTabSheet;
    TabSheet3: TTabSheet;
    procedure CaptureFileNameClick(Sender: TObject);
    procedure ResetSensorClick(Sender: TObject);
    procedure Run1Click(Sender: TObject);
    procedure Run4Click(Sender: TObject);
    procedure Run3Click(Sender: TObject);
    procedure SaveTempClick(Sender: TObject);
    procedure SetCommsClick(Sender: TObject);
    procedure SetWorkingDirClick(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure p2MouseLeave(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure p1MouseMove(Sender: TObject; Shift: TShiftState; X, Y: Integer);
    procedure p1Paint(Sender: TObject);
    procedure Timer1Timer(Sender: TObject);
    procedure TMinChange(Sender: TObject);
  private
    procedure DisplayImageData;
  end;

  Trec=record
    timestamp:Tdatetime;
    RejOnTime,SampleRate,BottleCount,RejectCount,exposure:integer;
    MaxDownSlope,AvgDetFill,SampleCount,DetectedFillLevel,algorithm,cap,neck,tollerance,tMin:byte;
    image:array[0..127] of byte;
  end;

  Tcfg=record
    Commport,baudrate,logoption:integer;
    DataDir:string[99];
  end;

var
  MainForm: TMainForm;
  ser: TBlockSerial;
  Rxbuf:byte;
  s:string;
  rc:integer;
  fxd:trec;
  fx:file of Trec;
  cfg:tcfg;
  cfgfile:file of tcfg;

implementation

{$R *.lfm}

{ TMainForm }

Procedure PurgeSerial;// purge buffer if we get to here
begin
//  if ser.WaitingData=0 then exit;
//  Mainform.log.Text:=Mainform.log.text+'Purged ';
//  while ser.WaitingData>0 do
//    Mainform.log.Text:=Mainform.log.text+' '+inttohex(ser.RecvByte(5),2);
  ser.Purge;
end;

Procedure SetStatus(s:string);
begin
  if Mainform.status.caption=' '+s+' ' then exit;
  Mainform.log.lines.add(s);
  Mainform.status.caption:=' '+s+' ';
end;

procedure TMainForm.Timer1Timer(Sender: TObject); // Receiva and process data
var
  i:integer;
begin
  timer1.Enabled:=false;
  try
    Floppy.tag:=floppy.tag+1;
    Comms.Tag:=comms.tag+1;
    Comms.Visible:=comms.Tag<3;
    Floppy.Visible:=floppy.tag<20;
    NoResponse.tag:=NoResponse.tag+1;
    if (fxd.cap=0) or (noResponse.tag mod 50=0) then ser.SendByte(5);
    NoResponse.Visible:=(NoResponse.tag>52) and (NoResponse.tag mod 10>5) and not CommErr.visible;
    Status.Visible:=(Status.caption<>'') and not CommErr.visible;
    if ser.WaitingData=0 then exit;
    Comms.Tag:=0;
    NoResponse.tag:=0;
    RxBuf:=ser.RecvByte(5);
    if RxBuf=1 then
    begin
      SetStatus('RE-BOOT..');
      exit;
    end;
    if RxBuf in [2,3] then
    begin
      if (RxBuf=2) and (fxd.BottleCount>0) then SetStatus('RUN');
      if (RxBuf=2) and (fxd.BottleCount=0) then SetStatus('CALIBRATE');
      if RxBuf=3 then SetStatus('RAW IMAGE CALIBRATION');
      if ser.RecvByte(5)=99 then // chk prefix byte
      begin
        for i:=0 to 127 do
          fxd.image[i] := ser.RecvByte(5);
        if ser.RecvByte(5)=99 then // chk postfix byte
        begin
          p1.Visible:=true;
          DisplayImagedata;
          exit;
        end;
      end;
    end;
    if RxBuf=4 then
    begin
      SetStatus('WAITING LINE RE-START');
      exit;
    end;
    if RxBuf=5 then
      if ser.RecvByte(5)=99 then
      begin
        fxd.tMin := ser.RecvByte(5);
        fxd.algorithm := ser.RecvByte(5);
        fxd.cap := ser.RecvByte(5);
        fxd.neck := ser.RecvByte(5);
        fxd.SampleRate := (ser.RecvByte(5)*256)+ser.RecvByte(5); // 2 bytes (MSB Txd First)
        fxd.RejOnTime := (ser.RecvByte(5)*256)+ser.RecvByte(5); // 2 bytes (MSB Txd First)
        fxd.tollerance := ser.RecvByte(5);
        TMin.Text:=inttostr(fxd.tMin);
        algorithm.Text:=inttostr(fxd.algorithm);
        cap.Text:=inttostr(fxd.cap);
        neck.Text:=inttostr(fxd.neck);
        SampleRate.Text:=inttostr(fxd.SampleRate);
        RejOnTime.Text:=inttostr(fxd.RejOnTime);
        Tollerance.Text:=inttostr(fxd.tollerance);
        SaveNVM.Enabled:=false;
        SaveTemp.Enabled:=false;
        SavePanel.Visible:=true;
        exit;
      end;
    if RxBuf=6 then
      if ser.RecvByte(5)=99 then
      begin
        fxd.AvgDetFill := ser.RecvByte(5);
        fxd.MaxDownslope := ser.RecvByte(5);
        fxd.SampleCount := ser.RecvByte(5);
        fxd.DetectedFillLevel := ser.RecvByte(5);
        fxd.Exposure := (ser.RecvByte(5)*256)+ser.RecvByte(5); // 2 bytes (MSB Txd First)
        fxd.BottleCount := (ser.RecvByte(5)*256*256)+(ser.RecvByte(5)*256)+ser.RecvByte(5); // 3 bytes (MSB Txd First)
        fxd.RejectCount := (ser.RecvByte(5)*256)+ser.RecvByte(5); // 2 bytes (MSB Txd First)
        AvgDetFill.Caption:=inttostr(fxd.AvgDetFill);
        Exposure.caption:=inttostr(fxd.Exposure);
        BottleCount.Caption:=inttostr(fxd.BottleCount);
        RejectCount.Caption:=inttostr(fxd.RejectCount);
        if fxd.BottleCount>0 then Status.color:=clmoneygreen else Status.color:=clyellow;
        log.lines.add(Inttostr(fxd.BottleCount)+':  Detected Fill Level: '+inttostr(fxd.DetectedFillLevel)+', Sample Count: '+inttostr(fxd.SampleCount)+', Max DownSlope: '+inttostr(fxd.MaxDownSlope));
        exit;
      end;
    if RxBuf in [9,10] then
    begin
      if RxBuf=9 then SetStatus('CALIBRATION (WAIT LINE START)');
      if RxBuf=10 then SetStatus('CALIBRATING SETTING EXPOSURE');
      fxd.Exposure:=(ser.RecvByte(5)*256)+ser.RecvByte(5); // 2 bytes (MSB Txd First)
      Exposure.caption:=inttostr(fxd.Exposure)+' [ '+inttostr(ser.RecvByte(5))+' : '+inttostr(ser.RecvByte(5))+' : '+inttostr(ser.RecvByte(5))+' ]';
      exit;
    end;
    PurgeSerial;
    Status.caption:='';
  finally
    timer1.Enabled:=true;
  end;
end;

procedure TMainForm.TMinChange(Sender: TObject);
begin
  SaveNVM.Enabled:=true;
  SaveTemp.Enabled:=true;
end;

procedure TMainForm.p1MouseMove(Sender: TObject; Shift: TShiftState; X, Y: Integer);
begin
  Info2.caption:='Y='+inttostr(Y div 4)+',   X='+inttostr(X div 2);
end;

procedure TMainForm.SetWorkingDirClick(Sender: TObject);
begin
  if not DirectoryExists(DataDir.text) then
  begin
    messageDLG('Folder does not exist',mterror,[mbok],0);
    exit;
  end;
  if log0.checked then cfg.logoption:=0;
  if log1.checked then cfg.logoption:=1;
  if log2.checked then cfg.logoption:=2;
  cfg.DataDir:=DataDir.text;
  rewrite(cfgfile);
  write(cfgfile,cfg);
  closefile(cfgfile);
  log.lines.add('Saved Default capture folder');
end;

procedure TMainForm.SetCommsClick(Sender: TObject);
begin
  cfg.baudrate:=Baudrate.itemindex;
  cfg.Commport:=Comport.ItemIndex;
  rewrite(cfgfile);
  write(cfgfile,cfg);
  closefile(cfgfile);
  log.lines.add('Saved Serial Settings.');
  Formshow(nil);
end;

procedure TMainForm.Run3Click(Sender: TObject);
begin
  ser.SendByte(3); // Raw image data
  pagecontrol1.ActivePage:=tabsheet1;
end;

procedure TMainForm.SaveTempClick(Sender: TObject);
begin
  ser.SendByte(7);
  ser.SendByte(fxd.tMin);
  ser.SendByte(fxd.algorithm);
  ser.SendByte(fxd.cap);
  ser.SendByte(fxd.neck);
  ser.SendByte(hi(fxd.SampleRate));
  ser.SendByte(lo(fxd.SampleRate));
  ser.SendByte(hi(fxd.RejOnTime));
  ser.SendByte(lo(fxd.RejOnTime));
  ser.SendByte(fxd.tollerance);
  if ser.RecvByte(500)=7 then
  begin
    SaveTemp.Enabled:=false;
    log.Lines.add('Saved Settings to Sensor');
    if sender=SaveNVM then
    begin
      ser.SendByte(8);
      if ser.RecvByte(2000)=8 then
      begin
        SaveNVM.Enabled:=false;
        log.Lines.add('Wrote Settings to Sensors Non-Volatile Memory');
      end;
    end;
  end;
end;

procedure TMainForm.Run4Click(Sender: TObject);
begin
  ser.SendByte(4); // End
  log.lines.add(inttostr(ser.RecvByte(500)));
end;

procedure TMainForm.ResetSensorClick(Sender: TObject);
begin
  ser.SendByte(9); // Reset
  FormShow(nil);
end;

procedure TMainForm.CaptureFileNameClick(Sender: TObject);
begin
  clipboard.AsText:=CaptureFilename.caption;
  messageDLG('Filename copied to clipboard',mtinformation,[mbok],0);
end;

procedure TMainForm.Run1Click(Sender: TObject);
begin
  ser.SendByte(1); // Raw image data
end;

procedure TMainForm.FormShow(Sender: TObject);
begin
  rc:=0;
  p1.Visible:=false;
  screen.Cursor:=crhourglass;
  BottleCount.caption:='0';
  RejectCount.caption:='0';
  Exposure.caption:='-';
  AvgDetFill.caption:='-';
  Status.caption:='';
  enabled:=false;
  try
    CommErr.Visible:=false;
    Info.caption:='';
    Info2.caption:='';
    log.text:=formatdatetime('dd-mmm-yyyy hh:nn',now)+cr+lf+cr+lf+'Reading sensorcontrol.cfg.. ';
    assignfile(cfgfile,'SensorControl.cfg');
    try
      // read in config
      Reset(cfgfile);
      read(cfgfile,cfg);
      Baudrate.itemindex:=cfg.baudrate;
      Comport.ItemIndex:=cfg.Commport;
      DataDir.text:=cfg.DataDir;
      log0.checked:=cfg.logoption=0;
      log1.checked:=cfg.logoption=1;
      log2.checked:=cfg.logoption=2;
      closefile(cfgfile);
      log.text:=log.text+' [OK]';
    except
      log.text:=log.text+' [FAIL - Using Defaults!]';
    end;
    log.text:=log.text+cr+lf+cr+lf+'Connecting to '+COMPort.text;
    try
      ser.Connect(COMPort.text);
      ser.Config(strtoint(BaudRate.text),8,'N',SB1,false,false);
      if ser.LastError<>0 then raise exception.Create(inttostr(ser.LastError)+', '+ser.LastErrorDesc);
      log.text:=log.text+' [OK]';
      if capturefilename.caption='' then
      begin
        rc:=0;
        CaptureFilename.caption:=cfg.DataDir+'Capture-Data-'+formatdatetime('yyddmmhhnn',now)+'.fls';
        try
          log.text:=log.text+cr+lf+cr+lf+'Opening '+CaptureFilename.caption+' for write.. ';
          assignfile(fx,Capturefilename.caption);
          rewrite(fx);
          log.text:=log.text+' [OK]'+cr+lf;
        except
          log.text:=log.text+' [FAIL!]'+cr+lf;
        end;
      end;
      Panel9.Visible:=true;
      tabsheet1.TabVisible:=true;
      pagecontrol1.ActivePage:=Tabsheet1;
      sleep(500);
      ser.SendByte(5); // Get settings
      sleep(300);
      ser.SendByte(6); // Get counters
    except
      on e:exception do
      begin
        CommErr.visible:=true;
        tabsheet1.TabVisible:=false;
        Panel9.Visible:=false;
        log.text:=log.text+' [FAIL Serial Error: '+trim(e.Message)+']';
        pagecontrol1.ActivePage:=Tabsheet3;
      end;
    end;
  finally
    enabled:=true;
    screen.Cursor:=crdefault;
  end;
end;

procedure TMainForm.DisplayImageData;
var
  y,max,min:byte;
begin
  max:=0;
  min:=$ff;
  for y:=0 to 127 do
  begin
    if fxd.image[y]>max then max:=fxd.image[y];
    if fxd.image[y]<min then min:=fxd.image[y];
  end;
  if captureFilename.caption<>'' then
  begin
    if log1.checked or (log2.checked and ((fxd.DetectedFillLevel<=fxd.AvgDetFill-fxd.tollerance) or (fxd.DetectedFillLevel>=fxd.AvgDetFill+fxd.tollerance))) then
    begin
      write(fx,fxd);
      inc(Rc);
      Info.Caption:='Record #'+inttostr(rc)+': Min='+inttostr(min)+', Max='+inttostr(max);
      floppy.tag:=0;
    end;
  end
  else
    Info.Caption:='Min='+inttostr(min)+', Max='+inttostr(max);
  p1Paint(p1);
end;

procedure TMainForm.p1Paint(Sender: TObject);   // erase then re-plot image from data array onto paint component
var
  y:byte;
begin
  p1.Canvas.brush.Color:=clblack;
  p1.Canvas.FillRect(0,0,p1.width,p1.height);
  for y:=0 to 127 do
  begin
    if (y>fxd.cap) and (y<fxd.neck) then p1.Canvas.brush.Color:=clgreen else p1.Canvas.brush.Color:=clmaroon;
    p1.canvas.fillRect(0, y*4, fxd.image[y]*2, (y+1)*4);
  end;
end;

procedure TMainForm.p2MouseLeave(Sender: TObject);
begin
  Info2.caption:='';
end;

procedure TMainForm.FormCreate(Sender: TObject);
begin
  CaptureFilename.caption:='';
  fxd.cap:=0;
  ser:=TBlockSerial.Create;
  MainForm.KeyPreview := True;
end;

procedure TMainForm.FormDestroy(Sender: TObject);
begin
  if captureFilename.caption<>'' then
  begin
    closefile(fx);
    if (rc<5) and log1.checked then deletefile(captureFilename.caption);
  end;
  ser.free;
end;


end.
