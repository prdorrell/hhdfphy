'''
-------------------------------------------------------------------------------
Copyright (c) 2011 Plextek Limited, Great Chesterford, England.
All Rights Reserved
-------------------------------------------------------------------------------
THIS FILE IS VERSION CONTROLLED USING SVN. DO NOT HAND EDIT THESE FIELDS
-------------------------------------------------------------------------------
$HeadURL: http://mantis/svn/projects/Spies/Ames/Src/x1S00/tags/Release_3.3/test/src/debug_gui.py $
$Revision: 6312 $
$Author: pdm $
$Date: 2011-07-15 09:39:50 +0100 (Fri, 15 Jul 2011) $
-------------------------------------------------------------------------------
File Description (User Field)
----------------------------

Provide a GUI for controlling debug functions.

-------------------------------------------------------------------------------
'''

print "loading libs...",

import  ConfigParser
import  sys
import  wx
import  wx.lib.masked   as masked

import  get_defaults
import  phyint
import  test_correlator_impulse
import  test_correlator_psch
import  test_correlator_ul
import  test_correlator_gains
import  test_correlator_averaging

import  test_detector_impulse
import  test_detector_psch
import  test_detector_ul

import  test_frontend_gains

import  test_ul_search

print "done.\n"

class MultiViewVariable:
    """ Class to store a variable that has multiple views """
    def __init__(self, value):
        self.value = value
        self.viewList = []
    
    def set(self, value):
        self.value = value
        self.updateViews()
    
    def get(self):
        return self.value
    
    def updateViews(self):
        for view in self.viewList:
            view.SetValue(self.value)
    
    def addView(self, view):
        self.viewList.append(view)
        self.updateViews()

class RedirectText(object):
    """ Class to redirect stdout/stderr to a text control """
    def __init__(self,aWxTextCtrl):
        self.out=aWxTextCtrl
 
    def write(self,string):
        self.out.WriteText(string)
        
class GeneralForm(wx.Panel):
    """ The form used for the general controls page """

    def __init__(self, *args, **kwargs):
        """ Initialise an instance of the class """
        # Call the initialisation functions of all parents
        super(GeneralForm, self).__init__(*args, **kwargs)
        
        # Prevent the recursion for the IP address
        self.updatingIpAddress = False
        
        # Create the controls, bind their events and handle the layout
        self.createControls()
        self.bindEvents()
        self.doLayout()

    def createControls(self):
        """ Create the controls """
        self.logger = wx.TextCtrl(self, style=wx.TE_MULTILINE|wx.TE_READONLY, size=(600, 300))
        font = wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.NORMAL, False, u'Courier New')
        self.logger.SetFont(font)
        
        self.ipAddressLabel = wx.StaticText(self, label="IP Address")
        self.ipAddressCtrl  = wx.TextCtrl(self, size=(80, -1))
        self.GetParent().GetParent().ipAddress.addView(self.ipAddressCtrl)
        self.connectDisconnectButton = wx.Button(self, label="Connect")

        self.gainLabel = wx.StaticText(self, label="Gain")
        self.gainCtrl  = wx.TextCtrl( self, -1, size=(80, -1), style=wx.TE_RIGHT )
        self.gainCtrl.SetValue("%d" % self.GetParent().GetParent().gain)
        self.setGainButton = wx.Button(self, label="Set Gain")

        self.bandLabel = wx.StaticText(self, label="Band")
        self.bandCtrl  = wx.ComboBox( self, -1)
        self.bandCtrl.Append("UL850")
        self.bandCtrl.Append("DL850")
        self.bandCtrl.Append("UL900")
        self.bandCtrl.Append("DL900")
        self.bandCtrl.Append("UL1700")
        self.bandCtrl.Append("DL1700")
        self.bandCtrl.Append("UL1800")
        self.bandCtrl.Append("DL1800")
        self.bandCtrl.Append("UL1900")
        self.bandCtrl.Append("DL1900")
        self.bandCtrl.Append("UL2100")
        self.bandCtrl.Append("DL2100")
        self.bandCtrl.SetValue(self.GetParent().GetParent().band)
        self.setBandButton = wx.Button(self, label="Set Band")

        self.freqLabel = wx.StaticText(self, label="Freq")
        self.freqCtrl  = wx.TextCtrl( self, -1, size=(80, -1), style=wx.TE_RIGHT )
        self.freqCtrl.SetValue("%d" % self.GetParent().GetParent().freq)
        self.setFreqButton = wx.Button(self, label="Set Freq")

        self.dacLabel = wx.StaticText(self, label="DAC")
        self.dacCtrl  = wx.TextCtrl( self, -1, size=(80, -1), style=wx.TE_RIGHT )
        self.dacCtrl.SetValue("%d" % self.GetParent().GetParent().dac)
        self.setDacButton = wx.Button(self, label="Set DAC")
        

    def bindEvents(self):
        """ Bind each control's events to the appropriate handler """
        for control, event, handler in \
            [(self.ipAddressCtrl,           wx.EVT_TEXT,   self.onIpAddressChanged),
             (self.connectDisconnectButton, wx.EVT_BUTTON, self.onConnectDisconnect),
             (self.setGainButton,           wx.EVT_BUTTON, self.onSetGainButton),
             (self.setBandButton,           wx.EVT_BUTTON, self.onSetBandButton),
             (self.setFreqButton,           wx.EVT_BUTTON, self.onSetFreqButton),
             (self.setDacButton,            wx.EVT_BUTTON, self.onSetDacButton)
             ]:
            control.Bind(event, handler)

    def doLayout(self):
        """ Layout the controls """

        # The connection controls form a horizontal row
        connectionSizer = wx.BoxSizer(orient=wx.HORIZONTAL)

        connectionSizer.Add(self.ipAddressLabel,          flag=(wx.ALIGN_CENTER_VERTICAL | wx.RIGHT), border=20)
        connectionSizer.Add(self.ipAddressCtrl,           flag=(wx.ALIGN_CENTER_VERTICAL | wx.EXPAND ))
        connectionSizer.Add(self.connectDisconnectButton, flag=(wx.ALIGN_CENTER_VERTICAL | wx.LEFT), border=20)
        
        # The gain, band, frequency and DAC controls form a grid
        controlsBoxSizer = wx.FlexGridSizer(rows=4, cols=3, vgap=10, hgap=10)

        controlsBoxSizer.Add(self.dacLabel,           flag=(wx.ALIGN_CENTER_VERTICAL | wx.RIGHT), border=20)
        controlsBoxSizer.Add(self.dacCtrl,            flag=(wx.ALIGN_CENTER_VERTICAL | wx.EXPAND ))
        controlsBoxSizer.Add(self.setDacButton,       flag=(wx.ALIGN_CENTER_VERTICAL))

        controlsBoxSizer.Add(self.bandLabel,          flag=(wx.ALIGN_CENTER_VERTICAL | wx.RIGHT), border=20)
        controlsBoxSizer.Add(self.bandCtrl,           flag=(wx.ALIGN_CENTER_VERTICAL | wx.EXPAND ))
        controlsBoxSizer.Add(self.setBandButton,      flag=(wx.ALIGN_CENTER_VERTICAL))

        controlsBoxSizer.Add(self.freqLabel,          flag=(wx.ALIGN_CENTER_VERTICAL | wx.RIGHT), border=20)
        controlsBoxSizer.Add(self.freqCtrl,           flag=(wx.ALIGN_CENTER_VERTICAL | wx.EXPAND ))
        controlsBoxSizer.Add(self.setFreqButton,      flag=(wx.ALIGN_CENTER_VERTICAL))

        controlsBoxSizer.Add(self.gainLabel,          flag=(wx.ALIGN_CENTER_VERTICAL | wx.RIGHT), border=20)
        controlsBoxSizer.Add(self.gainCtrl,           flag=(wx.ALIGN_CENTER_VERTICAL | wx.EXPAND ))
        controlsBoxSizer.Add(self.setGainButton,      flag=(wx.ALIGN_CENTER_VERTICAL))

        # A GridSizer will contain the other controls:
        boxSizer1 = wx.BoxSizer(orient=wx.VERTICAL)
        
        boxSizer1.Add(connectionSizer)
        boxSizer1.Add((0, 20))
        boxSizer1.Add(controlsBoxSizer)

        # A horizontal BoxSizer will contain the GridSizer (on the left)
        # and the logger text control (on the right):
        boxSizer2 = wx.BoxSizer(orient=wx.HORIZONTAL)
        
        boxSizer2.Add(boxSizer1, border=5, flag=wx.ALL)
        boxSizer2.Add(self.logger, border=5, flag=wx.ALL|wx.EXPAND, proportion=1)

        self.SetSizerAndFit(boxSizer2)

    # Callback methods:

    def onIpAddressChanged(self, event):
        if not self.updatingIpAddress :
            self.updatingIpAddress = True
            self.GetParent().GetParent().ipAddress.set(self.ipAddressCtrl.GetValue())
            self.updatingIpAddress = False
            
        event.Skip()

    def onConnectDisconnect(self, event):
        """ Handles the pressing of the Connect/Disconnect button """
        if not self.GetParent().GetParent().connected:
            ipAddress = self.GetParent().GetParent().ipAddress.get()
            self.phy = phyint.phyint(ipAddress, 55555)
            self.GetParent().GetParent().connected = True
            self.connectDisconnectButton.SetLabel("Disconnect")
            self.GetParent().GetParent().SetStatusText("FPGA version id: 0x%04x" % self.phy.debug_reg_read( 0 ), 0)
            self.GetParent().GetParent().SetStatusText("Connected", 1)
        else:
            self.phy.__del__()
            self.GetParent().GetParent().connected = False
            self.connectDisconnectButton.SetLabel("Connect")
            self.GetParent().GetParent().SetStatusText("", 0)
            self.GetParent().GetParent().SetStatusText("Idle", 1)

    def onSetGainButton(self, event):
        """ Handles the pressing of the Set Gain button """
        if not self.GetParent().GetParent().connected:
            dlg = wx.MessageDialog(self, "Must be connected to set the gain", " Error", wx.OK)
            dlg.ShowModal() # Shows it
            dlg.Destroy() # finally destroy it when finished.
            
        else:
            gainStr = self.gainCtrl.GetValue()
            try:
                gain = int(gainStr)
                
            except ValueError:
                dlg = wx.MessageDialog(self, "The gain must be a positive integer", " Error", wx.OK)
                dlg.ShowModal() # Shows it
                dlg.Destroy() # finally destroy it when finished.
                
            else:
                if gain < 0:
                    dlg = wx.MessageDialog(self, "The gain must be a positive integer", " Error", wx.OK)
                    dlg.ShowModal() # Shows it
                    dlg.Destroy() # finally destroy it when finished.
                    
                else:
                    self.GetParent().GetParent().gain = gain
                    self.phy.debug_set_gain(self.GetParent().GetParent().gain)

    def onSetFreqButton(self, event):
        """ Handles the pressing of the Set Freq button """
        if not self.GetParent().GetParent().connected:
            dlg = wx.MessageDialog(self, "Must be connected to set the frequency", " Error", wx.OK)
            dlg.ShowModal() # Shows it
            dlg.Destroy() # finally destroy it when finished.
            
        else:
            freqStr = self.freqCtrl.GetValue()
            try:
                freq = float(freqStr)
                
            except ValueError:
                dlg = wx.MessageDialog(self, "The frequency must be positive", " Error", wx.OK)
                dlg.ShowModal() # Shows it
                dlg.Destroy() # finally destroy it when finished.
                
            else:
                if freq < 0:
                    dlg = wx.MessageDialog(self, "The frequency must be positive", " Error", wx.OK)
                    dlg.ShowModal() # Shows it
                    dlg.Destroy() # finally destroy it when finished.
                    
                else:
                    #                   band,   min freq.,  max freq.,  AD_input, gpio, sw_band
                    lookupTable = {                                         \
                                    "UL850":  (  824000000,  849000000, "RXLBRF",  0x4, "GSM_LB_TX" ), \
                                    "DL850":  (  869000000,  894000000, "RXLBRF",  0x2, "GSM_LB_TX" ), \
                                    "UL900":  (  880000000,  915000000, "RXLBRF",  0xC, "WCDMA_1"   ), \
                                    "DL900":  (  925000000,  960000000, "RXLBRF",  0xA, "WCDMA_1"   ), \
                                    "UL1700": ( 1710000000, 1755000000, "RXHB1RF", 0x1, "GSM_HB_TX" ), \
                                    "DL1700": ( 2110000000, 2155000000, "RXHB2RF", 0xA, "WCDMA_3"   ), \
                                    "UL1800": ( 1710000000, 1785000000, "RXHB1RF", 0x1, "GSM_HB_TX" ), \
                                    "DL1800": ( 1805000000, 1880000000, "RXHB1RF", 0x2, "GSM_RX_2"  ), \
                                    "UL1900": ( 1850000000, 1910000000, "RXHB2RF", 0x4, "WCDMA_2"   ), \
                                    "DL1900": ( 1930000000, 1990000000, "RXHB2RF", 0x2, "WCDMA_2"   ), \
                                    "UL2100": ( 1920000000, 1980000000, "RXHB2RF", 0xC, "WCDMA_3"   ), \
                                    "DL2100": ( 2110000000, 2170000000, "RXHB2RF", 0xA, "WCDMA_3"   )}

                    #
                    #   Get the band and set it.  Since it is a list value it
                    #   must be valid. 
                    #
                    band = self.bandCtrl.GetValue()
                    self.phy.set_band(band)
                    self.GetParent().GetParent().band = band
                    
                    #
                    #   Get the settings for this band.  Since it is a list
                    #   value there must be an entry so an unhandled exception
                    #   is appropriate if it does not exist as a dictionary
                    #   key. 
                    #
                    (minFreq, maxFreq, AD_input, gpio, sw_band) = lookupTable[band]
                    if ((minFreq <= freq) & (freq <= maxFreq)):
                        self.GetParent().GetParent().freq = freq
                        
                        #
                        #   Set the switches and then the frequency.
                        #
                        self.phy.debug_switches( sw_band, AD_input, gpio )
                        if AD_input == "RXLBRF":
                            self.phy.debug_set_frequency( freq, band="low" )
                        else:
                            self.phy.debug_set_frequency( freq, band="high" )
                        
                    else:
                        dlg = wx.MessageDialog(self, "The frequency is not supported", " Error", wx.OK)
                        dlg.ShowModal() # Shows it
                        dlg.Destroy() # finally destroy it when finished.
#                    self.phy.debug_set_frequency(freq)

    def onSetBandButton(self, event):
        """ Handles the pressing of the Set Band button """
        if not self.GetParent().GetParent().connected:
            dlg = wx.MessageDialog(self, "Must be connected to set the band", " Error", wx.OK)
            dlg.ShowModal() # Shows it
            dlg.Destroy() # finally destroy it when finished.
            
        else:
            #
            #   Get the band and set it.  Since it is a list value it must be
            #   valid. 
            #
            self.GetParent().GetParent().band = self.bandCtrl.GetValue()
            self.phy.set_band(self.GetParent().GetParent().band)

    def onSetDacButton(self, event):
        """ Handles the pressing of the Set DAC button """
        if not self.GetParent().GetParent().connected:
            dlg = wx.MessageDialog(self, "Must be connected to set the DAC", " Error", wx.OK)
            dlg.ShowModal() # Shows it
            dlg.Destroy() # finally destroy it when finished.
            
        else:
            dacStr = self.dacCtrl.GetValue()
            try:
                dac = int(dacStr)
                
            except ValueError:
                dlg = wx.MessageDialog(self, "The DAC value must be a positive integer between 0 and 4095", " Error", wx.OK)
                dlg.ShowModal() # Shows it
                dlg.Destroy() # finally destroy it when finished.
                
            else:
                if dac < 0:
                    dlg = wx.MessageDialog(self, "The DAC value must be a positive integer between 0 and 4095", " Error", wx.OK)
                    dlg.ShowModal() # Shows it
                    dlg.Destroy() # finally destroy it when finished.
                    
                else:
                    self.GetParent().GetParent().dac = dac
                    self.phy.set_vctcxo(self.GetParent().GetParent().dac)
            
    # Helper method(s):

    def __log(self, message):
        """ Private method to append a string to the logger text control. """
        self.logger.AppendText('%s\n'%message)

class ScriptsForm(wx.Panel):
    """ The form used for the scripts page """

    def __init__(self, *args, **kwargs):
        """ Initialise an instance of the class """
        # Call the initialisation functions of all parents
        super(ScriptsForm, self).__init__(*args, **kwargs)
        
        # Prevent the recursion for the IP address
        self.updatingIpAddress = False
        
        # Create the controls, bind their events and handle the layout
        self.createControls()
        self.bindEvents()
        self.doLayout()
 

    def createControls(self):
        """ Create the controls """
        self.logger = wx.TextCtrl(self, style=wx.TE_MULTILINE|wx.TE_READONLY, size=(600, 300))
        font = wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.NORMAL, False, u'Courier New')
        self.logger.SetFont(font)

        self.ipAddressLabel = wx.StaticText(self, label="IP Address")
        self.ipAddressCtrl  = wx.TextCtrl(self, size=(80, -1))
        self.GetParent().GetParent().ipAddress.addView(self.ipAddressCtrl)
        
        self.testCorrImpulseButton   = wx.Button(self, label="Impulse")
        self.testCorrPschButton      = wx.Button(self, label="PSCH")
        self.testCorrUlButton        = wx.Button(self, label="Uplink")
        self.testCorrGainsButton     = wx.Button(self, label="Gains")
        self.testCorrAveragingButton = wx.Button(self, label="Averaging")
        
        self.testDetImpulseButton    = wx.Button(self, label="Impulse")
        self.testDetPschButton       = wx.Button(self, label="PSCH")
        self.testDetUlButton         = wx.Button(self, label="Uplink")
        
        self.testFrontendGainsButton = wx.Button(self, label="Gains")

    def bindEvents(self):
        """ Bind each control's events to the appropriate handler """
        for control, event, handler in \
            [(self.ipAddressCtrl,           wx.EVT_TEXT,   self.onIpAddressChanged),
             (self.testCorrImpulseButton,   wx.EVT_BUTTON, self.onTestCorrImpulse),
             (self.testCorrPschButton,      wx.EVT_BUTTON, self.onTestCorrPsch),
             (self.testCorrUlButton,        wx.EVT_BUTTON, self.onTestCorrUl),
             (self.testCorrGainsButton,     wx.EVT_BUTTON, self.onTestCorrGains),
             (self.testCorrAveragingButton, wx.EVT_BUTTON, self.onTestCorrAveraging),
             (self.testDetImpulseButton,    wx.EVT_BUTTON, self.onTestDetImpulse),
             (self.testDetPschButton,       wx.EVT_BUTTON, self.onTestDetPsch),
             (self.testDetUlButton,         wx.EVT_BUTTON, self.onTestDetUl),
             (self.testFrontendGainsButton, wx.EVT_BUTTON, self.onTestFrontendGains)
             ]:
            control.Bind(event, handler)

    def doLayout(self):
        """ Layout the controls """

        # A horizontal BoxSizer will contain the GridSizer for the controls (on the left)
        # and the logger text control (on the right):
        boxSizer = wx.BoxSizer(orient=wx.HORIZONTAL)
        
        # A GridSizer will contain the other controls:
        gridSizer = wx.FlexGridSizer(rows=5, cols=1, vgap=10, hgap=10)
        
        # A horizontal BoxSizer will contain the IP address control
        ipAddressSizer = wx.BoxSizer(orient=wx.HORIZONTAL)

        ipAddressSizer.Add(self.ipAddressLabel, flag=(wx.ALIGN_CENTER_VERTICAL | wx.RIGHT), border=20)
        ipAddressSizer.Add(self.ipAddressCtrl,  flag=(wx.ALIGN_CENTER_VERTICAL | wx.EXPAND ))
        
        # A vertical BoxSizer will contain the Correlator test buttons
        testCorrBox = wx.StaticBox(self, -1, "Correlator Tests")
        testCorrBoxSizer = wx.StaticBoxSizer(testCorrBox, orient=wx.VERTICAL)
        
        # A vertical BoxSizer will contain the Detector test buttons
        testDetBox = wx.StaticBox(self, -1, "Detector Tests")
        testDetBoxSizer = wx.StaticBoxSizer(testDetBox, orient=wx.VERTICAL)
        
        # A vertical BoxSizer will contain the Frontend test buttons
        testFrontendBox = wx.StaticBox(self, -1, "Frontend Tests")
        testFrontendBoxSizer = wx.StaticBoxSizer(testFrontendBox, orient=wx.VERTICAL)

        # Prepare some reusable arguments for calling sizer.Add():
        expandOption = dict(flag=wx.EXPAND)
        noOptions    = dict()
        emptySpace = ((0, 0), noOptions)

        # Add the controls to the sizers:
        testCorrBoxSizer.Add(self.testCorrImpulseButton,       flag=(wx.ALIGN_CENTER_HORIZONTAL | wx.LEFT), border=40)
        testCorrBoxSizer.Add(self.testCorrPschButton,          flag=(wx.ALIGN_CENTER_HORIZONTAL | wx.LEFT), border=40)
        testCorrBoxSizer.Add(self.testCorrUlButton,            flag=(wx.ALIGN_CENTER_HORIZONTAL | wx.LEFT), border=40)
        testCorrBoxSizer.Add(self.testCorrGainsButton,         flag=(wx.ALIGN_CENTER_HORIZONTAL | wx.LEFT), border=40)
        testCorrBoxSizer.Add(self.testCorrAveragingButton,     flag=(wx.ALIGN_CENTER_HORIZONTAL | wx.LEFT), border=40)
                                                               
        testDetBoxSizer.Add(self.testDetImpulseButton,         flag=(wx.ALIGN_CENTER_HORIZONTAL | wx.LEFT), border=40)
        testDetBoxSizer.Add(self.testDetPschButton,            flag=(wx.ALIGN_CENTER_HORIZONTAL | wx.LEFT), border=40)
        testDetBoxSizer.Add(self.testDetUlButton,              flag=(wx.ALIGN_CENTER_HORIZONTAL | wx.LEFT), border=40)

        testFrontendBoxSizer.Add(self.testFrontendGainsButton, flag=(wx.ALIGN_CENTER_HORIZONTAL | wx.LEFT), border=40)

        for control, options in \
                [(ipAddressSizer,       noOptions), \
                 (testCorrBoxSizer,     expandOption), \
                 (testDetBoxSizer,      expandOption), \
                 (testFrontendBoxSizer, expandOption)]:
            gridSizer.Add(control, **options)

        for control, options in \
                [(gridSizer, dict(border=5, flag=wx.ALL)),
                 (self.logger, dict(border=5, flag=wx.ALL|wx.EXPAND,
                    proportion=1))]:
            boxSizer.Add(control, **options)

        self.SetSizerAndFit(boxSizer)

    # Callback methods:

    def onIpAddressChanged(self, event):
        if not self.updatingIpAddress :
            self.updatingIpAddress = True
            self.GetParent().GetParent().ipAddress.set(self.ipAddressCtrl.GetValue())
            self.updatingIpAddress = False
            
        event.Skip()

    def runScript(self, script, *args):
        """ Runs a test script """
        if self.GetParent().GetParent().connected:
            dlg = wx.MessageDialog(self, "Must be disconnected to run a script", " Error", wx.OK)
            dlg.ShowModal() # Shows it
            dlg.Destroy() # finally destroy it when finished.
            
        else:
            reload(script)
            
            ipAddress = self.GetParent().GetParent().ipAddress.get()
            
            self.GetParent().GetParent().SetStatusText("Running %s.main(%s)" % (script.__name__, ipAddress), 0)
            self.GetParent().GetParent().SetStatusText("Script", 1)
            
            # redirect stdout to the text window
            old_stdout = sys.stdout
            redir = RedirectText(self.logger)
            sys.stdout = redir
            
            try :
                script.main(ipAddress, *args)
                
            except Exception as inst :
                cause = inst          # __getitem__ allows args to be unpacked directly
                print "    Exception occurred, cause = ", cause
            
            # restore stdout
            sys.stdout = old_stdout
            
            self.GetParent().GetParent().SetStatusText("", 0)
            self.GetParent().GetParent().SetStatusText("Idle", 1)

    def onTestCorrImpulse(self, event):
        """ Handles the pressing of the Correlator Impulse button """
        offsets      = [0, 38400, 76799]
        filter_len   = 256
        filter_delay = 0
        plot_results = True
        self.runScript(test_correlator_impulse, offsets, filter_len, filter_delay, plot_results)

        offsets      = [0, 38400, 76799]
        filter_len   = 2048
        filter_delay = 0
        plot_results = True
        self.runScript(test_correlator_impulse, offsets, filter_len, filter_delay, plot_results)

    def onTestCorrPsch(self, event):
        """ Handles the pressing of the Correlator PSCH button """
        offsets      = [0, 2560, 5119]
        plot_results = True
        self.runScript(test_correlator_psch, offsets, plot_results)

    def onTestCorrUl(self, event):
        """ Handles the pressing of the Correlator Uplink button """
        offsets      = [38400, 76798, 76799, 0, 1]
        plot_results = True
        self.runScript(test_correlator_ul, offsets, plot_results)

    def onTestCorrGains(self, event):
        """ Handles the pressing of the Correlator Gains button """
        signal_power = get_defaults.getDefaultHighSignalPower()

        plot_results = True
        self.runScript(test_correlator_gains, signal_power, plot_results)

        signal_power = get_defaults.getDefaultMidSignalPower()
        plot_results = True
        self.runScript(test_correlator_gains, signal_power, plot_results)

        signal_power = get_defaults.getDefaultLowSignalPower()
        plot_results = True
        self.runScript(test_correlator_gains, signal_power, plot_results)

    def onTestCorrAveraging(self, event):
        """ Handles the pressing of the Correlator Averaging button """
        offsets           = [0, 19200, 38400, 57600]
        filter_len        = 256
        frames_to_average = 4
        plot_results      = True
        self.runScript(test_correlator_averaging, offsets, filter_len, frames_to_average, plot_results)

        offsets           = [0, 19200, 38400, 57600]
        filter_len        = 2048
        frames_to_average = 4
        plot_results      = True
        self.runScript(test_correlator_averaging, offsets, filter_len, frames_to_average, plot_results)

    def onTestDetImpulse(self, event):
        """ Handles the pressing of the Detector Impulse button """
        self.runScript(test_detector_impulse)

    def onTestDetPsch(self, event):
        """ Handles the pressing of the Detector PSCH button """
        self.runScript(test_detector_psch)

    def onTestDetUl(self, event):
        """ Handles the pressing of the Detector Uplink button """
        self.runScript(test_detector_ul)

    def onTestFrontendGains(self, event):
        """ Handles the pressing of the Frontend Gains button """
        self.runScript(test_frontend_gains)

    # Helper method(s):

    def __log(self, message):
        """ Private method to append a string to the logger text control. """
        self.logger.AppendText('%s\n'%message)

class UplinkSearchForm(wx.Panel):
    """ The form used for the scripts page """

    def __init__(self, *args, **kwargs):
        """ Initialise an instance of the class """
        # Call the initialisation functions of all parents
        super(UplinkSearchForm, self).__init__(*args, **kwargs)
        
        # Prevent the recursion for the IP address
        self.updatingIpAddress = False
        
        # Create the controls, bind their events and handle the layout
        self.createControls()
        self.bindEvents()
        self.doLayout()
 

    def createControls(self):
        """ Create the controls """
        self.logger = wx.TextCtrl(self, style=wx.TE_MULTILINE|wx.TE_READONLY, size=(600, 300))
        font = wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.NORMAL, False, u'Courier New')
        self.logger.SetFont(font)

        self.ipAddressLabel = wx.StaticText(self, label="IP Address")
        self.ipAddressCtrl  = wx.TextCtrl(self, size=(80, -1))
        self.GetParent().GetParent().ipAddress.addView(self.ipAddressCtrl)
        
        self.runButton = wx.Button(self, label="Run")

    def bindEvents(self):
        """ Bind each control's events to the appropriate handler """
        for control, event, handler in \
            [(self.ipAddressCtrl, wx.EVT_TEXT,   self.onIpAddressChanged),
             (self.runButton,     wx.EVT_BUTTON, self.onRun)
             ]:
            control.Bind(event, handler)

    def doLayout(self):
        """ Layout the controls """

        # A horizontal BoxSizer will contain the GridSizer for the controls (on the left)
        # and the logger text control (on the right):
        boxSizer = wx.BoxSizer(orient=wx.HORIZONTAL)
        
        # A GridSizer will contain the other controls:
        gridSizer = wx.FlexGridSizer(rows=5, cols=1, vgap=10, hgap=10)
        
        # A horizontal BoxSizer will contain the IP address control
        ipAddressSizer = wx.BoxSizer(orient=wx.HORIZONTAL)

        ipAddressSizer.Add(self.ipAddressLabel, flag=(wx.ALIGN_CENTER_VERTICAL | wx.RIGHT), border=20)
        ipAddressSizer.Add(self.ipAddressCtrl,  flag=(wx.ALIGN_CENTER_VERTICAL | wx.EXPAND ))

        # Prepare some reusable arguments for calling sizer.Add():
        expandOption = dict(flag=wx.EXPAND)
        noOptions    = dict()
        emptySpace = ((0, 0), noOptions)

        # Add the controls to the sizers:
        for control, options in \
                [(ipAddressSizer,       noOptions), \
                 (self.runButton,       noOptions)]:
            gridSizer.Add(control, **options)

        for control, options in \
                [(gridSizer, dict(border=5, flag=wx.ALL)),
                 (self.logger, dict(border=5, flag=wx.ALL|wx.EXPAND,
                    proportion=1))]:
            boxSizer.Add(control, **options)

        self.SetSizerAndFit(boxSizer)

    # Callback methods:

    def onIpAddressChanged(self, event):
        if not self.updatingIpAddress :
            self.updatingIpAddress = True
            self.GetParent().GetParent().ipAddress.set(self.ipAddressCtrl.GetValue())
            self.updatingIpAddress = False
            
        event.Skip()

    def onRun(self, event):
        """ Handles the pressing of the Run button """
        if self.GetParent().GetParent().connected:
            dlg = wx.MessageDialog(self, "Must be disconnected to run a script", " Error", wx.OK)
            dlg.ShowModal() # Shows it
            dlg.Destroy() # finally destroy it when finished.
            
        else:
            reload(test_ul_search)
            
            ipAddress = self.GetParent().GetParent().ipAddress.get()
            
            self.GetParent().GetParent().SetStatusText("Running %s.main(%s)" % (test_ul_search.__name__, ipAddress), 0)
            self.GetParent().GetParent().SetStatusText("Script", 1)
            
            # redirect stdout to the text window
            old_stdout = sys.stdout
            redir = RedirectText(self.logger)
            sys.stdout = redir
            
            try :
                band       = "UL2100"
                freq_Hz    = 1930000000  
                code_num   = 0
                num_pilots = 8
                
                test_ul_search.main(ipAddress, band, freq_Hz, code_num, num_pilots)

            except Exception as inst :
                cause = inst          # __getitem__ allows args to be unpacked directly
                print "    Exception occurred, cause = ", cause
            
            # restore stdout
            sys.stdout = old_stdout
            
            self.GetParent().GetParent().SetStatusText("", 0)
            self.GetParent().GetParent().SetStatusText("Idle", 1)

    # Helper method(s):

    def __log(self, message):
        """ Private method to append a string to the logger text control. """
        self.logger.AppendText('%s\n'%message)

class MainFrame(wx.Frame):
    """ The main frame for the application """
    def __init__(self, *args, **kwargs):
        #
        #   Call the initialisation function for the base class.  The "super"
        #   function returns a proxy object that takes care of calling the
        #   parent class's __init__ function.  It has the benefit of not
        #   explicitly naming the parent class so that its type can be easily
        #   changed.  If multiple inheritance is being used it ensures that
        #   the __init__ function of both parents is called.
        #
        super(MainFrame, self).__init__(*args, **kwargs)

        #
        #    Open the configuration file, or create a new one if it does not
        #    exist.
        #
        self.config_filename = 'debug_gui.cfg'
        self.config = ConfigParser.RawConfigParser()
        files_read = self.config.read(self.config_filename)
        if len(files_read) == 0 :
            # create a configuration file.
            self.config.add_section('General Settings')
            ipAddress = '10.10.7.93'
            self.config.set('General Settings', 'IP Address', ipAddress)
    
            # Writing our configuration file to 'example.cfg'
            with open(self.config_filename, 'wb') as configfile :
                self.config.write(configfile)
                
        else :
            # read the settings.
            ipAddress = self.config.get('General Settings', 'IP Address')
        
        #
        #   Default to the disconnected state. 
        #
        self.connected = False
        
        #
        #   Setup the default IP address. 
        #
        self.ipAddress = MultiViewVariable(ipAddress)
        
        #
        #   Setup the default gain. 
        #
        self.gain = 100
        
        #
        #   Setup the default frequency. 
        #
        self.freq = 1920e6
        
        #
        #   Setup the default band. 
        #
        self.band = "UL2100"
        
        #
        #   Setup the default DAC value. 
        #
        self.dac = 0xfff/2
        
        #
        #   Create a notebook, which provides tabbed pages, and add a form to
        #   each page.
        #
        notebook = wx.Notebook(self)
        
        generalForm      = GeneralForm(notebook)
        scriptsForm      = ScriptsForm(notebook)
        uplinkSearchForm = UplinkSearchForm(notebook)
        
        notebook.AddPage(generalForm,      'General')
        notebook.AddPage(scriptsForm,      'Scripts')
        notebook.AddPage(uplinkSearchForm, 'UL Search')
        
        # The status bar displays the menu tips.
        self.CreateStatusBar(2)
        self.SetStatusWidths([-1, 100])
        self.SetStatusText("Idle", 1)
        
        #
        #   Set the frame to the right size manually.  This is feasible for
        #   the frame since the frame contains just one component. If the
        #   frame had contained more than one component sizers would be used.
        #
        self.SetClientSize(notebook.GetBestSize())
        
        #
        #    Bind the function to handle the window being closed.
        #
        self.Bind(wx.EVT_CLOSE, self.onCloseWindow)

    def onCloseWindow(self, event):
        """ Handles the closing of the window """
        #    Update the configurtion file
        self.config.set('General Settings', 'IP Address', self.ipAddress.get())
        with open(self.config_filename, 'wb') as configfile :
            self.config.write(configfile)
            
        self.Destroy()

def main() :
    """The main function of the GUI script"""
    app = wx.App(0)
    frame = MainFrame(None, title='Ames Debug GUI')
    frame.Show()
    app.MainLoop()                

if __name__ == '__main__':
    main()
