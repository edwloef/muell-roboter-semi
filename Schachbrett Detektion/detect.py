import cv2

#Vektoren zu den unteren beiden Ecken, von Mitte unten ausgehend (also aus "Robotersicht")
def find_chessboard_vector(image):

    found, corners = cv2.findChessboardCorners(image,(5,3))
    print(len(corners))

    image_width = image.shape[1]
    image_height = image.shape[0]

    if(found): #wenn detektiert

        #linkester Vektor unten und rechtester Vektor unten
        rechteste_idx = []
        rechteste_idx.append(0)

        linkeste_idx = []
        linkeste_idx.append(0)


        for i in range(1,len(corners)):

            #links, Threshold = 6   (wenn trotz Addition von 6 corners[i][0][0] trd. weiter links ist, dann soll sie die neue Vergleichscorner werden und die anderen werden gelöscht)
            if(corners[i][0][0]+6 < corners[linkeste_idx[0]][0][0]):
                linkeste_idx.clear()
                linkeste_idx.append(i)

            #(mit einem Threshold von 6 wird die corner noch in die Liste aufgenommen, auch wenn sie schlechter als die bisher beste ist)
            elif corners[i][0][0] - 6 <= corners[linkeste_idx[0]][0][0]:
                linkeste_idx.append(i)


            #rechts, Threshold = 6   
            if(corners[i][0][0]-6 > corners[rechteste_idx[0]][0][0]):
                rechteste_idx.clear()
                rechteste_idx.append(i)
                 
            #Threshold 6
            elif corners[i][0][0]+6 >= corners[rechteste_idx[0]][0][0]:
                rechteste_idx.append(i)

        #unter den rechtesten und linkesten die untersten raussuchen

        left_corner_idx = linkeste_idx[0]
        for i in range(1, len(linkeste_idx)):
            if(corners[linkeste_idx[i]][0][1] > corners[left_corner_idx][0][1]):
                left_corner_idx = i

        right_corner_idx = rechteste_idx[0]
        for i in range(1, len(rechteste_idx)):
            if(corners[rechteste_idx[i]][0][1] > corners[right_corner_idx][0][1]):
                right_corner_idx = i


        

      #  cv2.line(image,(tuple(corners[left_corner_idx][0])), (int(image_width/2),image_height), (100,100,255), 10)
       # cv2.line(image,(tuple(corners[right_corner_idx][0])), (int(image_width/2),image_height), (100,100,255), 10)
        #cv2.imshow("Detection (red)",image)
        #cv2.waitKey(0) 
        #cv2.destroyAllWindows() 


        #Vektoren erstellen                             #width            #height
        left_vec = (int(corners[left_corner_idx][0][0] - image_width/2), int(image_height- corners[left_corner_idx][0][1]))
        right_vec = (int(corners[right_corner_idx][0][0] - image_width/2), int(image_height- corners[left_corner_idx][0][1]))

        return [left_vec,right_vec]
        
    else:
        return []


image = cv2.imread("./images/test1.jpg")

vecs = find_chessboard_vector(image) #DAS MUSS VON DER ZENTRALE AUS DANN AUFGERUFEN WERDEN (man könnte den Rest ja in eine andere py Datei auslagern und die Methode als Modul der py Datei importieren)

#print(vecs)

