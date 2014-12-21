/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
 
#include "msp430.h"
#include "BCUart.h"

#include "USB_config/descriptors.h"
#include "USB_app/usbConstructs.h"

extern WORD rxByteCount;                        // Momentarily stores the number of bytes received
extern BYTE buf_bcuartToUsb[BC_RXBUF_SIZE];     // Same size as the UART's rcv buffer
extern BYTE buf_usbToBcuart[128];               // This can be any size


// Receive buffer for the UART.  Incoming bytes need a place to go immediately,
// otherwise there might be an overrun when the next comes in.  The USCI ISR
// puts them here.
uint8_t  bcUartRcvBuf[BC_RXBUF_SIZE];

// The index within bcUartRcvBuf, where the next byte will be written.
uint16_t bcUartRcvBufIndex = 0;

// Boolean flag indicating whether bcUartRcvBufIndex has reached the
// threshold BC_RX_WAKE_THRESH.  0 = FALSE, 1 = TRUE
uint8_t  bcUartRxThreshReached = 0;


// Initializes the USCI_A0 module as a UART, using baudrate settings in
// bcUart.h.  The baudrate is dependent on SMCLK speed.
void bcUartInit(void)
{
    // Always use the step-by-step init procedure listed in the USCI chapter of
    // the F5xx Family User's Guide
    UCA0CTL1 |= UCSWRST;        // Put the USCI state machine in reset	//KJ:
    UCA0CTL1 |= UCSSEL__SMCLK;  // Use SMCLK as the bit clock	//KJ:

    // Set the baudrate
    UCA0BR0 = UCA0_BR0;	//KJ:
    UCA0BR1 = UCA0_BR1;	//KJ:
    UCA0MCTL = (UCA0_BRF << 4) | (UCA0_BRS << 1) | (UCA0_OS);	//KJ:

    P3SEL |= BIT3+BIT4;         // Configure these pins as TXD/RXD

    P2DIR |= BIT6;	//KJ: EAP_RX_ACK_CONFIG();
	P2OUT |= BIT6;	//KJ: EAP_RX_ACK_SET();

	P2DIR &= ~BIT3;	//KJ: EAP_TX_ACK_CONFIG();
	P2IES |= BIT3;
	P2IFG &= ~BIT3;
	P2IE |= BIT3;

	// suspend the MCM
	P2OUT &= ~BIT6;	//KJ: EAP_RX_ACK_CLR();

    UCA0CTL1 &= ~UCSWRST;       // Take the USCI out of reset	//KJ:
    UCA0IE |= UCRXIE;           // Enable the RX interrupt.  Now, when bytes are	//KJ:
                                // rcv'ed, the USCI_A1 vector will be generated.
}


// Sends 'len' bytes, starting at 'buf'
void bcUartSend(uint8_t * buf, uint8_t len)
{
    uint8_t i = 0;

    // Write each byte in buf to USCI TX buffer, which sends it out
    while (i < len)
    {
        UCA0TXBUF = *(buf+(i++));	//KJ:

        // Wait until each bit has been clocked out...
        while(!(UCTXIFG==(UCTXIFG & UCA0IFG))&&((UCA0STAT & UCBUSY)==UCBUSY));	//KJ:
    }
}


// Copies into 'buf' whatever bytes have been received on the UART since the
// last fetch.  Returns the number of bytes copied.
uint16_t bcUartReceiveBytesInBuffer(uint8_t* buf)
{
    uint16_t i, count;

    // Hold off ints for incoming data during the copy
    UCA0IE &= ~UCRXIE;	//KJ:

    for(i=0; i<bcUartRcvBufIndex; i++)
    {
        buf[i] = bcUartRcvBuf[i];
    }

    count = bcUartRcvBufIndex;
    bcUartRcvBufIndex = 0;     // Move index back to the beginning of the buffer
    bcUartRxThreshReached = 0;

    // Restore USCI interrupts, to resume receiving data.
    UCA0IE |= UCRXIE;	//KJ:

    return count;
}



// The USCI_A0 receive interrupt service routine (ISR).  Executes every time a
// byte is received on the back-channel UART.
#pragma vector=USCI_A0_VECTOR	//KJ:
__interrupt void bcUartISR(void)
{
    bcUartRcvBuf[bcUartRcvBufIndex++] = UCA0RXBUF;  // Fetch the byte, store	//KJ:
                                                    // it in the buffer.

    P2OUT &= ~BIT6;	//KJ: EAP_RX_ACK_CLR();
    P2OUT |= BIT6;	//KJ: EAP_RX_ACK_SET();

    // Wake main, to fetch data from the buffer.
    if(bcUartRcvBufIndex >= BC_RX_WAKE_THRESH)
    {
        bcUartRxThreshReached = 1;
        __bic_SR_register_on_exit(LPM3_bits);       // Exit LPM0-3
    }
}




#pragma vector=PORT2_VECTOR	//KJ: EAP_TX_ACK_VECTOR
__interrupt void txAckIsr(void)
{
    if (P2IFG & BIT3)	//KJ: EAP_TX_ACK_TST()
    {
        // Look for received bytes over USB. If any, send over backchannel UART.
        rxByteCount = cdcReceiveDataInBuffer(buf_usbToBcuart, sizeof(buf_usbToBcuart), CDC0_INTFNUM);
        //rxByteCount = hidReceiveDataInBuffer(buf_usbToBcuart, sizeof(buf_usbToBcuart), HID0_INTFNUM);
        if(rxByteCount)
        {
            bcUartSend(buf_usbToBcuart, rxByteCount);
        }
    }
    __bic_SR_register_on_exit(LPM3_bits);	//KJ: WAKEUP();
}

