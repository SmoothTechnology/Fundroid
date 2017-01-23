VERSION 4.00
Begin VB.Form Form1 
   Caption         =   "Form1"
   ClientHeight    =   8205
   ClientLeft      =   1170
   ClientTop       =   1770
   ClientWidth     =   11880
   Height          =   8610
   Left            =   1110
   LinkTopic       =   "Form1"
   ScaleHeight     =   8205
   ScaleWidth      =   11880
   Top             =   1425
   Width           =   12000
   Begin VB.Timer Timer1 
      Interval        =   120
      Left            =   1320
      Top             =   0
   End
   Begin VB.PictureBox Picture1 
      Height          =   7935
      Left            =   0
      Picture         =   "FORM1.frx":0000
      ScaleHeight     =   7875
      ScaleWidth      =   11715
      TabIndex        =   1
      Top             =   240
      Width           =   11775
   End
   Begin VB.CommandButton Command1 
      Caption         =   "Command1"
      Height          =   255
      Left            =   120
      TabIndex        =   0
      Top             =   0
      Width           =   1095
   End
   Begin VB.Label CursorY 
      Caption         =   ":"
      Height          =   255
      Left            =   5640
      TabIndex        =   5
      Top             =   0
      Width           =   1335
   End
   Begin VB.Label CursorX 
      Caption         =   ":"
      Height          =   255
      Left            =   4200
      TabIndex        =   4
      Top             =   0
      Width           =   1335
   End
   Begin VB.Label Buffer 
      Caption         =   ":"
      Height          =   255
      Left            =   7080
      TabIndex        =   3
      Top             =   0
      Width           =   3735
   End
   Begin MSCommLib.MSComm MSComm1 
      Left            =   2760
      Top             =   0
      _Version        =   65536
      _ExtentX        =   847
      _ExtentY        =   847
      _StockProps     =   0
      CDTimeout       =   0
      CommPort        =   1
      CTSTimeout      =   0
      DSRTimeout      =   0
      DTREnable       =   -1  'True
      Handshaking     =   0
      InBufferSize    =   1024
      InputLen        =   0
      Interval        =   1000
      NullDiscard     =   0   'False
      OutBufferSize   =   512
      ParityReplace   =   "?"
      RThreshold      =   0
      RTSEnable       =   0   'False
      Settings        =   "9600,n,8,1"
      SThreshold      =   0
   End
   Begin VB.Label Label1 
      Caption         =   "Count "
      Height          =   255
      Left            =   2040
      TabIndex        =   2
      Top             =   0
      Width           =   975
   End
End
Attribute VB_Name = "Form1"
Attribute VB_Creatable = False
Attribute VB_Exposed = False
Private Const OFS_MAXPATHNAME = 128

Private Type OFSTRUCT
        cBytes As Byte
        fFixedDisk As Byte
        nErrCode As Integer
        Reserved1 As Integer
        Reserved2 As Integer
        szPathName(OFS_MAXPATHNAME) As Byte
End Type

Private Declare Function OpenFile _
Lib "kernel32" _
(ByVal lpFileName As String, _
lpReOpenBuff As OFSTRUCT, _
ByVal wStyle As Long) As Long

Private Type SECURITY_ATTRIBUTES
        nLength As Long
        lpSecurityDescriptor As Long
        bInheritHandle As Boolean
End Type

Private Declare Function CreateFile _
Lib "kernel32" Alias "CreateFileA" _
(ByVal lpFileName As String, _
ByVal dwDesiredAccess As Long, _
ByVal dwShareMode As Long, _
lpSecurityAttributes As SECURITY_ATTRIBUTES, _
ByVal dwCreationDisposition As Long, _
ByVal dwFlagsAndAttributes As Long, _
ByVal hTemplateFile As Long) As Long

Private Declare Function WriteFile Lib _
"kernel32" _
(ByVal hFile As Long, _
lpBuffer As Any, _
ByVal nNumberOfBytesToWrite As Long, _
lpNumberOfBytesWritten As Long, _
ByVal lpOverlapped As Long) As Long

Private Declare Function ReadFile Lib _
"kernel32" _
(ByVal hFile As Long, _
lpBuffer As Any, _
ByVal nNumberOfBytesToRead As Long, _
lpNumberOfBytesRead As Long, _
ByVal lpOverlapped As Any) As Long

Private Declare Sub Sleep Lib _
"kernel32" _
(ByVal dwMilliseconds As Long)

Private Declare Function GetLastError _
Lib "kernel32" () As Long


Private Sub Command1_Click()
'  MSComm1.CommPort = 1
'  MSComm1.Settings = "9600,N,8,1"
'  MSComm1.RThreshold = 1
'  MSComm1.InputBufferSize = 8
'  MSComm1.PortOpen = True



'
'  Dim hSerial As Long
'  Dim buffer(1 To 16) As Byte
'  Dim totalBytes As Long
'  Dim ret As Integer
'  Dim Text As String
'  Dim of As OFSTRUCT
'  ' Dim sec As SECURITY_ATTRIBUTES
'  ' sec.nLength = 0
'  ' hSerial = CreateFile("c:\hello.txt", GENERIC_READ, FILE_SHARE_READ, sec, OPEN_EXISTING, 0, 0)
'  hSerial = OpenFile("COM1", of, OF_EXIST Or OF_READ)
'  If hSerial = -1 Then
'    ret = Err.LastDllError()
'  End If
'  ' Sleep (2000)
'  ret = ReadFile(hSerial, buffer(1), 16, totalBytes, 0&)
'  Text = StrConv(buffer, vbUnicode)
'  Debug.Print "'" & Text & "'"
'  MsgBox ("got")
'  MsgBox (Text)
  
  
  ' MsgBox (buffer)
  
End Sub

Private Sub Form_Load()
  MSComm1.CommPort = 1
  MSComm1.Settings = "9600,N,8,1"
  MSComm1.RThreshold = 1
  MSComm1.InBufferSize = 1024
  MSComm1.InputLen = 128
  MSComm1.PortOpen = True
End Sub

Private Sub MSComm1_OnComm()
  Dim read As String
  Dim sofar As String
  Dim first As Integer
  Dim X As Integer
  Dim Y As Integer
  Dim ending As Integer
  read = ""
  sofar = buffer.Caption
  With MSComm1
    Select Case .CommEvent
      Case comEvReceive
        read = .Input
        buffer.Caption = buffer.Caption & read
        ending = InStr(1, buffer.Caption, Chr(10))
        If ending > 10 Then
          buffer.Caption = ""
          ending = 0
        End If
        If ending > 0 Then
          first = InStr(1, buffer.Caption, ",")
          If first > 0 And first < Len(buffer.Caption) Then
              X = Val(Mid(buffer.Caption, 1, first - 1))
              Y = Val(Mid(buffer.Caption, first + 1, ending - 1))
              CursorX.Caption = X
              CursorY.Caption = Y
          End If
          buffer.Caption = ""
          MSComm1.InBufferCount = 0
        End If
    End Select
  End With
End Sub


Private Sub Timer1_Timer()
  Static Count As Integer
  Dim Text As String
  Dim Part As String
  Dim Part2 As String
  Dim Num As Integer
  Dim X As Integer
  Dim Y As Integer
  Dim twipScale As Integer
  Dim circleRadius As Integer
  ' Count up each frame.
  Count = Count + 1
  'Label1.Caption = Count
  
  ' Flash a hello message.
  'If Count Mod 10 > 7 Then
  '  Picture1.Picture = LoadPicture("c:\code\fundroid\hello.bmp")
  'Else
  
    twipScale = 15
    circleRadius = 200
    If CursorX.Caption = ":" Then
    ' pass
    Else
      X = Val(CursorX.Caption)
      Y = Val(CursorY.Caption)
      ' Load the map image.
      Picture1.Picture = LoadPicture("c:\code\fundroid\map.bmp")
      ' Draw a large circle on it.
      Picture1.FillColor = vbRed
      Picture1.FillStyle = vbSolid
      Picture1.Circle (X * twipScale, Y * twipScale), circleRadius, vbRed
    End If
  'End If
End Sub

