import acapture
import cv2




def main():
    print("Hello World!")

    cap = acapture.open(0) #/dev/video0
    print("successfully opened camera device!")

    while True:
        check,frame = cap.read() # non-blocking
        if check:
            print("sucessfully read a frame")
            frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
            cv2.imwrite('test.jpg', frame)

            #cv2.imshow("test",frame)
            #cv2.waitKey(1)

    
    

if __name__ == "__main__":
    main()
