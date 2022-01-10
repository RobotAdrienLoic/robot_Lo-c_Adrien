using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using ExtendedSerialPort;
using System.Windows.Threading;
using MouseKeyboardActivityMonitor.WinApi;
using MouseKeyboardActivityMonitor;
using System.Windows.Forms;
using Utilities;
using SciChart.Charting.Visuals;



namespace Interfacerobot
{
    public partial class MainWindow : Window
    {

        ReliableSerialPort serialPort1;
        DispatcherTimer timerAffichage;
        Robot robot = new Robot();
        private readonly KeyboardHookListener m_KeyboardHookManager;

        byte message_pid;
        

        public MainWindow()
        {
            // Set this code once in App.xaml.cs or application startup
            SciChartSurface.SetRuntimeLicenseKey("1A1VKrxdPNwYSgOimkTcwnomSYGyup+XohhyGusuhfUB5xGgZGMP1OZsLsIXS4Ep2tjbrdXpBcYYMdJ0pFBcCh0+s7IrrB7yvmnWbq2Cac+OpBsKgSn7Jg0Pmcew8opzoF+OYGjdsurmQvf86Z+zqmWV0zswF1hw9+871xp3+FP0/5sM5cwtDQROlQip/+QX7clJBNxs0epBpiXjhNaCCCx8oHYv/zA0/0uRR9OvSqjpNAOfZgdA+JKrCS/syOecWlK2WJas1G8bttFhGSci1jiJzIkkckuCB0wUZQkza81IJKvtl9BVwF740dBdKowNnHaxHG8Xqj3GDb80sNUuEz6pzRI/uuMPlp7j9qdSfXXXUfxLDE0Zu2kS8Fnwe2PQCWXZE/WZoJfxWfNUGAkv1Np3M4huFlSpupERH4y8oupclzE+JqGa6vQJqrtUr77JSsiJJnOIU5TB1TVFHsxQIFGQ41iXXwT5soq3tgJSontOfZOOhlXCTBgRolvC+5ZR3w==");
            InitializeComponent();
            serialPort1 = new ReliableSerialPort("COM3", 115200, Parity.None, 8, StopBits.One);
            serialPort1.DataReceived += SerialPort1_DataReceived;
            serialPort1.Open();

            timerAffichage = new DispatcherTimer();
            timerAffichage.Interval = new TimeSpan(0, 0, 0, 0, 100);
            timerAffichage.Tick += TimerAffichage_Tick;
            timerAffichage.Start();
            m_KeyboardHookManager = new KeyboardHookListener(new GlobalHooker());
            m_KeyboardHookManager.Enabled = true;
            m_KeyboardHookManager.KeyDown += HookManager_KeyDown;
            oscilloSpeed.AddOrUpdateLine(50, 200, "PosX");
            oscilloSpeed.ChangeLineColor(50, Colors.Red);
            oscilloSpeed.AddOrUpdateLine(52, 200, "PosY");
            oscilloSpeed.ChangeLineColor(52, Colors.Blue);

        }



        private void TimerAffichage_Tick(object sender, EventArgs e)
        {
            byte[] message = Encoding.ASCII.GetBytes("Bonjour");
            /*TextBoxReception.Text += robot.ReceivedText;
            robot.ReceivedText = "";*/
            while (robot.byteListReceived.Count > 0)
            {
                DecodeMessage(robot.byteListReceived.Dequeue());
                //TextBoxReception.Text += robot.byteListReceived.Dequeue();
                //byte byteReceived = robot.byteListReceived.Dequeue();
                //string receivedText = "0x" + byteReceived.ToString("X2") + " " ;
                //char receivedText = Convert.ToChar(byteReceived);
                // TextBoxReception.Text += receivedText;
            }

        }

        private void SerialPort1_DataReceived(object sender, DataReceivedArgs e)
        {
            //robot.ReceivedText += Encoding.UTF8.GetString(e.Data, 0, e.Data.Length);
            for (int i = 0; i < e.Data.Length; i++)
            {
                DecodeMessage(e.Data[i]);
                //robot.byteListReceived.Enqueue(e.Data[i]);
            }
        }


        #region Encodage/Decodage

        StateReception rcvState = StateReception.Waiting;
        int msgDecodedFunction = 0;
        int msgDecodedPayloadLength = 0;
        byte[] msgDecodedPayload;
        int msgDecodedPayloadIndex = 0;

        public enum StateReception
        {
            Waiting,
            FunctionMSB,
            FunctionLSB,
            PayloadLengthMSB,
            PayloadLengthLSB,
            Payload,
            CheckSum
        }

        public enum Functions
        {
            LEDS = 0x0020,
            telemetres = 0x0030,
            moteurs = 0x0040,
            message = 0x0080,
            RobotState=0x0050,
            Clavier=0x0053,
            Odometrie=0x0061,
            Asservissement=0x0075
        }

        public enum StateRobot
        {
            STATE_ATTENTE=0,
            STATE_ATTENTE_EN_COURS=1,
            STATE_AVANCE=2,
            STATE_AVANCE_EN_COURS=3,
            STATE_TOURNE_GAUCHE=4,
            STATE_TOURNE_GAUCHE_EN_COURS=5,
            STATE_TOURNE_DROITE=6,
            STATE_TOURNE_DROITE_EN_COURS=7,
            STATE_TOURNE_SUR_PLACE_GAUCHE=8,
            STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS=9,
            STATE_TOURNE_SUR_PLACE_DROITE=10,
            STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS=11,
            STATE_ARRET=12,
            STATE_ARRET_EN_COURS=13,
            STATE_RECULE=14,
            STATE_RECULE_EN_COURS=15,
            STATE_TOURNE_GAUCHE_DOUX =16,
            STATE_TOURNE_GAUCHE_DOUX_EN_COURS= 17,
            STATE_TOURNE_DROITE_DOUX=19,
            STATE_TOURNE_DROITE_DOUX_EN_COURS=18 ,
            STATE_DROITE_TUNNEL =20,
            STATE_DROITE_TUNNEL_EN_COURS= 21,
            STATE_GAUCHE_TUNNEL= 22,
            STATE_GAUCHE_TUNNEL_EN_COURS= 23,
        }

        byte CalculateChecksum(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            byte checksum = 0;
            checksum ^= 0xFE;
            checksum ^= (byte)(msgFunction >> 8);
            checksum ^= (byte)(msgFunction >> 0);
            checksum ^= (byte)(msgPayloadLength >> 8);
            checksum ^= (byte)(msgPayloadLength >> 0);
            for (int i = 0; i < msgPayload.Length; i++)
            {
                checksum ^= msgPayload[i];
            }
            return checksum;
        }

        void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            byte[] trame = new byte[6 + msgPayload.Length];
            int pos = 0;
            int i;
            trame[pos++] = 0xFE;
            trame[pos++] = (byte)(msgFunction >> 8);
            trame[pos++] = (byte)(msgFunction >> 0);
            trame[pos++] = (byte)(msgPayloadLength >> 8);
            trame[pos++] = (byte)(msgPayloadLength >> 0);
            for (i = 0; i < msgPayload.Length; i++)
            {
                trame[pos++] = msgPayload[i];
            }
            trame[pos++] = CalculateChecksum(msgFunction, msgPayloadLength, msgPayload);
            serialPort1.Write(trame, 0, pos);
        }

        private void DecodeMessage(byte c)
        {
            switch (rcvState)
            {
                case StateReception.Waiting:
                    if (c == 0xFE)
                    {
                        rcvState = StateReception.FunctionMSB;
                    }
                    msgDecodedFunction = 0;
                    msgDecodedPayloadIndex = 0;
                    msgDecodedPayloadLength = 0;
                    break;

                case StateReception.FunctionMSB:
                    msgDecodedFunction = (byte)(c << 8);
                    rcvState = StateReception.FunctionLSB;
                    break;

                case StateReception.FunctionLSB:
                    msgDecodedFunction += (byte)(c << 0);
                    rcvState = StateReception.PayloadLengthMSB;
                    break;

                case StateReception.PayloadLengthMSB:
                    msgDecodedPayloadLength = (byte)(c << 8);
                    rcvState = StateReception.PayloadLengthLSB;
                    break;

                case StateReception.PayloadLengthLSB:
                    msgDecodedPayloadLength += (byte)(c << 0);
                    if (msgDecodedPayloadLength == 0)
                    {
                        msgDecodedPayload = new byte[1];
                        rcvState = StateReception.CheckSum;
                    }
                    else if (msgDecodedPayloadLength != 0)
                    {
                        msgDecodedPayload = new byte[msgDecodedPayloadLength];
                        msgDecodedPayloadIndex = 0;
                        rcvState = StateReception.Payload;
                    }
                    else
                    {
                        rcvState = StateReception.CheckSum;
                    }
                    break;

                case StateReception.Payload:
                    msgDecodedPayload[msgDecodedPayloadIndex] = c;
                    msgDecodedPayloadIndex++;
                    if (msgDecodedPayloadIndex >= msgDecodedPayloadLength)
                    {
                        rcvState = StateReception.CheckSum;
                    }
                    break;

                case StateReception.CheckSum:
                    byte calculatedChecksum, receivedChecksum;
                    receivedChecksum = c;
                    calculatedChecksum = CalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
                    if (calculatedChecksum == receivedChecksum)
                    {
                        Dispatcher.Invoke(delegate { ProcessDecodedMessage(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload); });
                        
                    }
                    else
                    {
                        Dispatcher.Invoke(delegate { TextBoxReception.Text += "\n\r/!\\ Attention, message corrompu. fonctions: " + "0x" + msgDecodedFunction.ToString("X4"); });
                    }
                    rcvState = StateReception.Waiting;
                    break;

                default:
                    rcvState = StateReception.Waiting;
                    break;
            }
        }

        void ProcessDecodedMessage(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            int numLed, etatLed, IRGauche, IRCentre, IRDroite, MG, MD;
            float xpos, ypos, angleRadian, vitesseLineaire, vitesseAngulaire,temps;

            switch((Functions)msgFunction)
            {
                case Functions.LEDS:
                    numLed = msgPayload[0];
                    etatLed = msgPayload[1];
                    if (numLed == 1)
                    {
                        if (etatLed == 1)
                        {
                            //CheckBoxLed1.IsChecked = true;
                        }
                        else
                        {
                            //CheckBoxLed1.IsChecked = false;
                        }
                    }
                    else if (numLed == 2)
                    {
                        if (etatLed == 1)
                        {
                            //CheckBoxLed2.IsChecked = true;
                        }
                        else
                        {
                            //CheckBoxLed2.IsChecked = false;
                        }
                    }
                    else if (numLed == 3)
                    {
                        if (etatLed == 1)
                        {
                            //CheckBoxLed3.IsChecked = true;
                        }
                        else
                        {
                            //CheckBoxLed3.IsChecked = false;
                        }
                    }
                break;

                case Functions.telemetres:
                    IRDroite = msgPayload[0];
                    IRCentre = msgPayload[1];
                    IRGauche = msgPayload[2];
                    TextBoxTelemetres.Text = "IR Gauche:" + IRGauche.ToString() + "cm\n" + "IR Centre:" + IRCentre.ToString() + "cm\n" + "IR Droite:" + IRDroite.ToString() + "cm";
                break;

                case Functions.moteurs:
                    MG = msgPayload[0];
                    MD = msgPayload[1];
                    TextBoxMoteurs.Text = "Vitesse Gauche:" + MG.ToString() + "%\n" + "Vitesse Droite:" + MD.ToString() + "%\n";
                break;

                case Functions.message:
                    TextBoxReception.Text += Encoding.UTF8.GetString(msgPayload, 0, msgPayload.Length) + "\n";
                break;

                case Functions.RobotState:
                    int instant = (((int)msgPayload[1]) << 24) + (((int)msgPayload[2]) << 16) + (((int)msgPayload[3]) << 8) + (((int)msgPayload[4]));
                    //RtbReception.Text = ((StateRobot)(msgPayload[0])).ToString() + "\n\rTemps: " + instant.ToString() + " ms";
                break;

                case Functions.Odometrie:
                    byte[] tab = msgPayload.GetRange(0, 4);
                    temps = tab.GetFloat();
                    tab = msgPayload.GetRange(4, 4);
                    xpos = tab.GetFloat();
                    tab = msgPayload.GetRange(8, 4);
                    ypos = tab.GetFloat();
                    tab = msgPayload.GetRange(12, 4);
                    angleRadian = tab.GetFloat();
                    tab = msgPayload.GetRange(16, 4);
                    vitesseLineaire = tab.GetFloat();
                    tab = msgPayload.GetRange(20, 4);
                    vitesseAngulaire = tab.GetFloat();
                    xPos.Text = "Position en X : " + (xpos).ToString();
                    yPos.Text = "Position en Y : " + (ypos).ToString();
                    angleRD.Text = "Angle en radian : " + (angleRadian*180/Math.PI).ToString();
                    vLineaire.Text = "Vitesse linéaire : " + (vitesseLineaire).ToString();
                    vAngulaire.Text = "Vitesse angulaire : " + (vitesseAngulaire).ToString();
                    //TextBoxReception.Text += "\n\r le temps vaut " + temps;
                    //oscilloSpeed.AddPointToLine(50, temps,xpos);
                    break;

                case Functions.Asservissement:
                    int nb_octet = 0;
                    //-------------------
                    double consigneX;
                    double consigneTheta;
                    double valueX;
                    double valueTheta;
                    double errorX;
                    double errorTheta;
                    double commandX;
                    double commandTheta;
                    //-------------------
                    double corrPX;
                    double corrPTheta;
                    double corrIX;
                    double corrITheta;
                    double corrDX;
                    double corrDTheta;
                    //-------------------
                    double KpX;
                    double KpTheta;
                    double KiX;
                    double KiTheta;
                    double KdX;
                    double KdTheta;
                    //-------------------
                    double corrLimitPX;
                    double corrLimitPTheta;
                    double corrLimitIX;
                    double corrLimitITheta;
                    double corrLimitDX;
                    double corrLimitDTheta;
                    //------------------- Récupération des valeurs
                    byte[] tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    consigneX = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    consigneTheta = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    valueX = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    valueTheta = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    errorX = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    errorTheta = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    commandX = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    commandTheta = tabl.GetFloat();
                    //------------------- 
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    corrPX = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    corrPTheta = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    corrIX = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    corrITheta = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    corrDX = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    corrDTheta = tabl.GetFloat();
                    //-------------------
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    KpX = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    KpTheta = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    KiX = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    KiTheta = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    KdX = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    KdTheta = tabl.GetFloat();
                    //-------------------
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    corrLimitPX = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    corrLimitPTheta = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    corrLimitIX = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    corrLimitITheta = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    corrLimitDX = tabl.GetFloat();
                    tabl = msgPayload.GetRange(nb_octet, 4); nb_octet = nb_octet + 4;
                    corrLimitDTheta = tabl.GetFloat();

                    asservSpeedDisplay.UpdatePolarOdometrySpeed(valueX, valueTheta);

                    asservSpeedDisplay.UpdatePolarSpeedConsigneValues(consigneX, consigneTheta);
                    asservSpeedDisplay.UpdatePolarSpeedCommandValues(commandX, commandTheta);
                    
                    asservSpeedDisplay.UpdatePolarSpeedErrorValues(errorX, errorTheta);
                    asservSpeedDisplay.UpdatePolarSpeedCorrectionValues(corrPX, corrPTheta, corrIX, corrITheta, corrDX, corrDTheta);
                    asservSpeedDisplay.UpdatePolarSpeedCorrectionGains(KpX, KpTheta, KiX, KiTheta, KdX, KdTheta);
                    asservSpeedDisplay.UpdatePolarSpeedCorrectionLimits(corrLimitPX, corrLimitPTheta, corrLimitIX, corrLimitITheta, corrLimitDX, corrLimitDTheta);
                    break;

            }
        }

        bool autoControlActivated = false;
        private void HookManager_KeyDown(object sender, System.Windows.Forms.KeyEventArgs e)
        {
            if (autoControlActivated == true)
            {
                switch (e.KeyCode)
                {
                    case Keys.Left:
                        UartEncodeAndSendMessage(0x0051,1,new byte[] { (byte)StateRobot.STATE_TOURNE_SUR_PLACE_GAUCHE });
                        //textBoxClavier.Text = "Gauche";
                        break;
                    case Keys.Right:
                        UartEncodeAndSendMessage(0x0051, 1, new byte[] { (byte)StateRobot.STATE_TOURNE_SUR_PLACE_DROITE });
                        //textBoxClavier.Text = "Droite";
                        break;
                    case Keys.Up:
                        UartEncodeAndSendMessage(0x0051, 1, new byte[] { (byte)StateRobot.STATE_AVANCE });
                        //textBoxClavier.Text = "Avance";
                        break;
                    case Keys.Down:
                        UartEncodeAndSendMessage(0x0051, 1, new byte[] { (byte)StateRobot.STATE_RECULE });
                        //textBoxClavier.Text = "Recule";
                        break;
                    case Keys.PageDown:
                        UartEncodeAndSendMessage(0x0051, 1, new byte[] { (byte)StateRobot.STATE_ARRET });
                        //textBoxClavier.Text = "Arret";
                        break;
                }
            }
        }

        #endregion

        #region Boutons 
        private void textBox_TextChanged(object sender, TextChangedEventArgs e) 
        {
        }

        private void buttonEnvoyer_Click(object sender, RoutedEventArgs e)
        {
            robot.ReceivedText = null;
            TextBoxEmission.Text = "";
            TextBoxReception.Text = "";

        }

        private void sendMessage()
        {
            
            //TextBoxReception.Text = TextBoxReception.Text + "Message reçu: " + TextBoxEmission.Text + "\r\n";
            //TextBoxEmission.Text = "";
            serialPort1.Write(TextBoxEmission.Text);
            TextBoxEmission.Text = "";
        }

        private void buttonTest_Click(object sender, RoutedEventArgs e)
        {
            // byte[] message = Encoding.ASCII.GetBytes("Bonjour");

            //// ProcessDecodedMessage(0x0020, 2, new byte[] { 3,1 });
            //UartEncodeAndSendMessage(0x0020, 2, new byte[] { 3, 1 });   //led 3 true
            //UartEncodeAndSendMessage(0x0030, 3, new byte[] { 25, 30, 25 });  //IR 25cm 30cm 25cm
            //UartEncodeAndSendMessage(0x0040, 2, new byte[] { 41, 38 });  // Moteur1 41% Moteur2 38%
            ////UartEncodeAndSendMessage(0x0080, 7, message);
            //UartEncodeAndSendMessage(0x0020, 2, new byte[] { 1, 1 });   //led 1 true
            //UartEncodeAndSendMessage(0x0020, 2, new byte[] { 2, 1 });   //led 2 true        
            //// UartEncodeAndSendMessage(0x0080, (UInt16)message.Length, message);
            // UartEncodeAndSendMessage(0x0053, 1, new byte[] { 4 });
            // UartEncodeAndSendMessage(0x0050, 5, new byte[] { 2, 0, 0, 0, 10 });
            // UartEncodeAndSendMessage(0x0061, 5, new byte[] { 0, 0, 0, 0, 0 });


           byte [] correcteur = new byte[28];


            correcteur.SetValueRange(((float)(message_pid)).GetBytes(), 0);
            correcteur.SetValueRange(((float)(Convert.ToDecimal(Kp.Text))).GetBytes(), 4);
            correcteur.SetValueRange(((float)(Convert.ToDecimal(Ki.Text))).GetBytes(), 8);
            correcteur.SetValueRange(((float)(Convert.ToDecimal(Kd.Text))).GetBytes(), 12);
            correcteur.SetValueRange(((float)(Convert.ToDecimal(proportionnelleMax.Text))).GetBytes(), 16);
            correcteur.SetValueRange(((float)(Convert.ToDecimal(integralMax.Text))).GetBytes(), 20);
            correcteur.SetValueRange(((float)(Convert.ToDecimal(deriveeMax.Text))).GetBytes(), 24);

            UartEncodeAndSendMessage(0x0063, correcteur.Length, correcteur);

        }

        private void TextBoxEmission_KeyUp(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                //sendMessage();
                string message = TextBoxEmission.Text;
                byte[] array = Encoding.ASCII.GetBytes(message);
                UartEncodeAndSendMessage(0x0080, (UInt16)array.Length, array);
                TextBoxEmission.Text = "";
            }
        }

        bool choix_pid = true;
        private void buttonPid_Click(object sender, RoutedEventArgs e)
        {
            if (!choix_pid)
            {
                buttonPid.Content = "Pid X";
                message_pid = 1; //PidX

                choix_pid = true;
            }
            else
            {
                buttonPid.Content = "Pid T";
                message_pid = 0; //PidTheta

                choix_pid = false;
            }
        }

        bool compteur_etat = true;
        private void buttonControl_Click(object sender, RoutedEventArgs e)
        {            
            if (!compteur_etat){
                buttonControl.Content = "Automatique";
                autoControlActivated = false;
                compteur_etat = true;
                UartEncodeAndSendMessage(0x0052, 1, new byte[] {1});
                //textBoxClavier.Text = "";
            }
            else
            {         
                buttonControl.Content = "Manuel";
                autoControlActivated = true;
                compteur_etat = false;
                UartEncodeAndSendMessage(0x0052, 1, new byte[] {0});
            }
        }

        #endregion

        private void buttonReset_Click(object sender, RoutedEventArgs e)
        {
            UartEncodeAndSendMessage(0x0062,1, new byte[] {0});
        }
    }
}

