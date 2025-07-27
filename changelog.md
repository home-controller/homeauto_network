# Change Log

## 0.1.x

    0.1.0
    Minimally working with added delay between sends.
>
    0.1.1
        1. Added bit stuffing.
>
    0.1.2
        1. Added ISR using pin change interrupt to receive messages, Used GPT chat and untested
        2. Used a timer to check for  message end/line free
        3. This is all untested.
>   
    0.1.3
        - Changed CRC from 4 bits to 8
        - Added timer1 read method
        - Changed MessageId byte length from code to be 1 or 2 bytes depending on number of data bytes
        - Bit stuffing removed from ack fields.
        - Add extra dominant delimiter bit before EOF, to make pin change IRC reading work better.
        - Changed main.cpp to be a link to one of the examples in the examples folder.
        - Adding timer1(inTimer1.cpp) polling of network pin to read incoming messages
