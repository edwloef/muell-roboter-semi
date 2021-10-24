import os
import numpy as np
import skimage.io as io
import skimage.transform as transform
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image

NUM_CLASSES = 2 #proof of concept erstmal --> erstmal nur 2 klassen (plastic, paper)
NUM_IMAGES_TO_CREATE = 100


x_images = np.zeros(shape = (NUM_IMAGES_TO_CREATE, 448,448,3))

def convert_global_to_local(x_max_global, y_max_global,x_min_global, y_min_global): #bildgröße = 448x448, 64x64 = size of one cell (7x7 grid)

    
    #width and hight of the bounding box
    width_conv = (x_max_global-x_min_global)/64
    height_conv = (y_max_global-y_min_global)/64

    cell_colum = int((x_min_global+(width_conv*64/2))/64)+1 #der midpoint ist dafür entscheidend, ob das objekt zu der zelle gehört
    cell_row = int((y_min_global+(height_conv*64/2))/64)+1

    #center of the bounding box
    x_local = ((x_min_global/64) + width_conv/2) - int(((x_min_global/64) + width_conv/2)) 
    y_local = ((y_min_global/64) + height_conv/2) - int(((y_min_global/64) + height_conv/2))

    

    return cell_colum, cell_row, x_local, y_local, width_conv, height_conv

def getForegroundMask(foreground):
   # print(foreground.shape)
    mask_new = foreground.copy()[:,:,0]
    mask_new[mask_new>0] = 1
   # print(mask_new)
    return mask_new

def compose(foreground, mask, background):
    # resize background
    background = transform.resize(background, foreground.shape[:2])
    

    # Subtract the foreground area from the background
   # print(mask.shape)
    background = background*(1 - mask.reshape(foreground.shape[0], foreground.shape[1], 1))


    # Finally, add the foreground
    composed_image = background + foreground
    
    return composed_image

def foregroundAug(foreground):
    # Random rotation, zoom, translation
    angle = np.random.randint(-10,10)*(np.pi/180.0) 
    zoom = np.random.random()*0.3 + 0.3 # Zoom 
    t_x = np.random.randint(0, int(foreground.shape[1]/3))*1.7  #shiften horizontal
    t_y = np.random.randint(0, int(foreground.shape[0]/3))*1.7  #shiften vertikal

    tform = transform.AffineTransform(scale=(zoom,zoom),
                                rotation=angle,
                                translation=(t_x, t_y))
    foreground = transform.warp(foreground, tform.inverse)

    # Random horizontal flip with 0.5 probability
    if(np.random.randint(0,100)>=50):
        foreground = foreground[:, ::-1]
        
    return foreground



if __name__ == "__main__":

 #Struktur für information (daraus wird später y_true gemacht): information{image{garbage, garbage, garbage...}, image{garbage, garbage....}}
 #also ne Liste namens information, die Listen namens image (bzw sub_list) enthält, die Listen namens garbage (bzw sub_sub_list) enthält, die informationen über den Müll auf dem jeweiligen Bild enthalten
 information = [] 
    
 for j in range(0, NUM_IMAGES_TO_CREATE):

    sub_list = []   #images


     # Random selection of background from the backgrounds folder
    background_fileName = np.random.choice(os.listdir("./images/backgrounds/"))
    background = io.imread('./images/backgrounds/'+background_fileName)/255.0

    num_garbage_items = np.random.randint(1,5) 

    for i in range(num_garbage_items):

        garbage_category = np.random.randint(0,NUM_CLASSES) #ist im Moment auf plastic und paper beschränkt

   

        if garbage_category == 0:
            image_fileName = np.random.choice(os.listdir("./images/garbage/plastic/"))
            I = io.imread('./images/garbage/plastic/'+image_fileName)

        elif  garbage_category == 1:
            image_fileName = np.random.choice(os.listdir("./images/garbage/paper/"))
            I = io.imread('./images/garbage/paper/'+image_fileName)
            
        elif  garbage_category == 2:
            image_fileName = np.random.choice(os.listdir("./images/garbage/metal/"))
            I = io.imread('./images/garbage/metal/'+image_fileName)
            
        #von 1920x1080 auf 1920x1920 erweitern und dann auf 448x448 runterskalieren
        black_array = np.zeros(shape = (1920,1920,3))
        black_array = black_array.astype(np.uint8)
        black_array[:I.shape[0],:I.shape[1],:I.shape[2]] = I
        I = transform.resize(black_array, (448,448,3))






        I = foregroundAug(I)

    


        mask_new = getForegroundMask(I)

        

        background = compose(I, mask_new, background) #eigentlich sollte das aber hier nicht mehr unbedingt "background" heißen, sondern eher "background_für_den_nächsten_müll_der_eingefügt_wird"

        nz = np.nonzero(mask_new)  
        
        bbox = [np.min(nz[0]), np.min(nz[1]), np.max(nz[0]), np.max(nz[1])] #dort, wo die Farbe des "original" Müllbilds nicht schwarz ist, die größten und kleinsten x und y Werte raussuchen
                                                                            #--> daraus ergibt sich die bounding box

        x = bbox[1] #kleinster x wert
        y = bbox[0] #kleinster y wert
        width = bbox[3] - bbox[1]
        height = bbox[2] - bbox[0]

        #hier direkt in die lokalen Werte umwandeln 

        (cell_colum, cell_row, x_local, y_local, width_local, height_local) = convert_global_to_local(x+width, y+height, x, y)


        sub_sub_list = []   #information about garbage

        sub_sub_list.append(garbage_category)

        sub_sub_list.append(cell_colum)
        sub_sub_list.append(cell_row)

        sub_sub_list.append(x_local)
        sub_sub_list.append(y_local)

        sub_sub_list.append(width_local)
        sub_sub_list.append(height_local)



        sub_list.append(sub_sub_list)


    
    information.append(sub_list)

    #grid aufs bild zeichnen
    #for f in range(0,6):

       # for ff in range(448):

        #    for fff in range(3):

         #       background[64+f*64][ff][fff] = 0

          #      background[ff][64+f*64][fff] = 0



    io.imsave('./images/generated_images/'+str(j)+'.jpg', background) #das ist das komplette bild, nicht nur der background vom anfang
                                   
    x_images[j] = background

    # Display the image
   # plt.imshow(background)
    
    #draw bbox on the image

    #ich kann irgendwie nur eine bounding box zeichnen, wenn ich die codezeile noch für eine weitere/andere box wiederhole, wird die irgendwie trotzdem nicht angezeigt
    #ist ja aber auch egal, das verfahren, wie die box erstellt wird, ist immer gleich --> müsste immer richtig sein

    # raute hier unten entfernen, um die bounding box von EINEM Müll anzeigen zu lassen
    #plt.gca().add_patch(Rectangle((xs[0],ys[0]),widths[0],heights[0],linewidth=1,edgecolor='green',facecolor='none'))   #x,y = oben links (also die kleinesten Koordinaten)
    
   # plt.axis('off')
   # plt.show()


y_true = np.zeros(shape=(NUM_IMAGES_TO_CREATE, 588))   #(batch_size, infos) infos = S x S x predictions_per_cell
    

print(information)
for image in range(0, NUM_IMAGES_TO_CREATE):
    
    for garbage in range(len(information[image])):

        #7 ist die Anzahl der Zellen pro Reihe, hier wird sozusagen aus reihe und 
        #spalte (2dimensionen) errechnet, wo die box dann im eindimensionalen ("geflatteten") Vektor liegt

        #vektor_idx = (reihe-1)*7+spalte
        pos_in_vector = (information[image][garbage][1]+((information[image][garbage][2]-1)*7)) * 12 #12 = anzahl der infos pro zelle

        #class im one hot vector format
        y_true[image][pos_in_vector+information[image][garbage][0]] = 1

        #die informationen einfach an die stellen von beiden bounding boxen schreiben, weil man nicht weiß, welche box sich wie spezialisiert (ist von iou abhängig)
      
        y_true[image][pos_in_vector+NUM_CLASSES] = 1 #probability that there is an obj

        y_true[image][pos_in_vector+NUM_CLASSES+1] = information[image][garbage][3] #x_local
        y_true[image][pos_in_vector+NUM_CLASSES+2] = information[image][garbage][4] #y_local
        y_true[image][pos_in_vector+NUM_CLASSES+3] = information[image][garbage][5] #width_local
        y_true[image][pos_in_vector+NUM_CLASSES+4] = information[image][garbage][6] #height_local

       

            
        y_true[image][pos_in_vector+NUM_CLASSES+5] = 1 #probability that there is an obj

        y_true[image][pos_in_vector+NUM_CLASSES+6] = information[image][garbage][3] #x_local
        y_true[image][pos_in_vector+NUM_CLASSES+7] = information[image][garbage][4] #y_local
        y_true[image][pos_in_vector+NUM_CLASSES+8] = information[image][garbage][5] #width_local
        y_true[image][pos_in_vector+NUM_CLASSES+9] = information[image][garbage][6] #height_local




print(y_true)




np.save("./numpy arrays/x_images.npy", x_images)
np.save("./numpy arrays/y_true.npy", y_true)