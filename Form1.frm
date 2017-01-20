VERSION 4.00
Begin VB.Form Form1 
   Caption         =   "Form1"
   ClientHeight    =   4140
   ClientLeft      =   1305
   ClientTop       =   1665
   ClientWidth     =   6690
   Height          =   4545
   Left            =   1245
   LinkTopic       =   "Form1"
   ScaleHeight     =   4140
   ScaleWidth      =   6690
   Top             =   1320
   Width           =   6810
   Begin VB.Timer Timer1 
      Interval        =   120
      Left            =   1320
      Top             =   0
   End
   Begin VB.PictureBox Picture1 
      Height          =   3495
      Left            =   120
      Picture         =   "Form1.frx":0000
      ScaleHeight     =   3435
      ScaleWidth      =   6435
      TabIndex        =   1
      Top             =   600
      Width           =   6495
   End
   Begin VB.CommandButton Command1 
      Caption         =   "Command1"
      Height          =   495
      Left            =   120
      TabIndex        =   0
      Top             =   0
      Width           =   1095
   End
   Begin VB.Label Label1 
      Caption         =   "Count "
      Height          =   375
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
ByVal lpOverlapped As Long) As Long

Private Declare Sub Sleep Lib _
"kernel32" _
(ByVal dwMilliseconds As Long)

Private Declare Function GetLastError _
Lib "kernel32" () As Long


Private Sub Command1_Click()
  Dim hSerial As Long
  Dim buffer(1 To 16) As Byte
  Dim totalBytes As Long
  Dim ret As Long
  Dim text As String
  Dim of As OFSTRUCT
  ' Dim sec As SECURITY_ATTRIBUTES
  ' sec.nLength = 0
  ' hSerial = CreateFile("c:\hello.txt", GENERIC_READ, FILE_SHARE_READ, sec, OPEN_EXISTING, 0, 0)
  hSerial = OpenFile("c:\hello.txt", of, OF_EXIST Or OF_READ)
  If hSerial = -1 Then
    ret = Err.LastDllError()
  End If
  ' Sleep (2000)
  ret = ReadFile(hSerial, buffer(1), 16, totalBytes, 0&)
  text = StrConv(buffer, vbUnicode)
  Debug.Print "'" & text & "'"
  MsgBox ("got")
  MsgBox (text)
  
  
  ' MsgBox (buffer)
  
End Sub

Private Sub Timer1_Timer()
  Static Count As Integer
  Count = Count + 1
  Label1.Caption = Count
End Sub


