#!/usr/bin/env python3 -W ignore::DeprecationWarning
# main.py

async def main(gr, sr, mq):      
    # starting tasks concurrently
    await asyncio.gather(sr.run_asl(), gr.run(), mq.run())

if __name__ == "__main__":
    '''
    Let the program wait awhile before exiting.
    '''
    from gesture import gesture_recognition
    from asl import static_recognition
    from mqtt import myMqtt

    import asyncio

    # TODO: uncomment to hide warnings
    import warnings
    warnings.filterwarnings("ignore") 

    mq = myMqtt()
    gr = gesture_recognition(mq)
    sr = static_recognition(mq)

    try:
        asyncio.run(main(gr, sr, mq))
    except KeyboardInterrupt:
        sr.disconnect()
        gr.disconnect()
        mq.disconnect()
    finally:
        print('Exiting program.')

