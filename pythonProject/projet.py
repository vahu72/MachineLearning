import timeit
import pandas as pd
import tensorflow as tf
from keras.src.regularizers import l2

import matplotlib.pyplot as plt
import numpy as np
from keras import optimizers
from keras.layers import Dense, InputLayer, Flatten
from keras.models import Sequential
from keras.utils import plot_model  # install graphviz on OS
from sklearn.model_selection import train_test_split
# fix random seed for reproducibility
from tensorflow.python.framework.random_seed import set_random_seed

seed = 0
np.random.seed(seed)
set_random_seed(seed)

# Chemin du fichier csv
path = '/home/vahu72/Documents/ESEO/Machine_Learning/Projet/dataset/dataset.csv'
donnees = pd.read_csv(path)

# Initialiser les listes pour stocker les lots de données et les sorties attendues
E = []
Y = []

# Diviser les données en lots de 100 lignes avec la même sortie attendue
for sortie_attendue, groupe in donnees.groupby('Dimension'):
    # Diviser le groupe en lots de 100 lignes
    for i in range(0, len(groupe), 100):
        lot = groupe.iloc[i:i+100, :3]  # Sélectionner les colonnes x, y, z
        if len(lot) == 100:  # S'assurer que le lot a exactement 100 lignes
            E.append(lot.values)
            Y.append(sortie_attendue)

# Convertir les listes en tableaux numpy
E = np.array(E)
Y = np.array(Y)

# Afficher les dimensions des tableaux résultants
print("Dimensions des lots de données:", E.shape)
print("Dimensions des sorties attendues:", Y.shape)
plt.figure()
plt.plot(E[0, :])
plt.plot(E[1, :])
plt.plot(Y)
plt.legend(('X', 'Y', 'Z', 'Sortie attendue'))

# Data cleaning, need to transpose E

E_train, E_test, Y_train, Y_test = train_test_split(E, Y, test_size=0.20, random_state=seed)

# create model
#model = Sequential()
#model.add(InputLayer(input_shape=(100, 3)))
#model.add(Flatten())
#model.add(Dense(128, activation='sigmoid'))
#model.add(Dense(64, activation='sigmoid'))
#model.add(Dense(1, activation='sigmoid'))

l2_regularizer = l2(0.01)
model = tf.keras.models.Sequential([
    tf.keras.layers.Dense(100, activation='relu', input_shape=(E_train.shape[1], E_train.shape[2])),
    tf.keras.layers.Dropout(0.165),
    tf.keras.layers.Dense(50, activation='relu', kernel_regularizer=l2_regularizer),
    tf.keras.layers.Dropout(0.1),
    tf.keras.layers.Dense(25, activation='relu', kernel_regularizer=l2_regularizer),
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(1, activation='sigmoid')
])

opt = tf.keras.optimizers.RMSprop(momentum=0.1885)#optimizers.SGD(learning_rate=0.34, momentum=0.2)
model.compile(loss='mean_squared_error', optimizer=opt, metrics=['acc'])
# print model in .png file
plot_model(model)

#train
start_time = timeit.default_timer()
history = model.fit(E_train, Y_train, validation_split=0.15, shuffle=False, epochs=200, verbose=0, batch_size=20)
print("Temps passé : %.2fs" % (timeit.default_timer() - start_time))

#plot figure
plt.figure()
plt.plot(history.history['acc'])
plt.plot(history.history['val_acc'])
plt.title('Model accuracy')
plt.ylabel('Accuracy')
plt.xlabel('Epoch')
plt.legend(['Train', 'Val'], loc='upper left')
# plt.show()
# Plot training & validation loss values

plt.figure()
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('Model loss')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.legend(['Train', 'Val'], loc='upper left')
# plt.show()

# evaluate the model
scores = model.evaluate(E_test, Y_test)
print("\nEvaluation sur le test data %s: %.2f - %s: %.2f%% " % (
    model.metrics_names[0], scores[0], model.metrics_names[1], scores[1] * 100))

#evaluate with the all dataset and plot
prediction = model.predict_on_batch(E)
prediction = prediction.reshape(71, 1)
attendues = Y.reshape(71, 1)

plt.figure()
plt.subplot(1, 2, 1)
plt.imshow(attendues, extent=[0, 1, 0, 1])
plt.title('Cartopgrahie de la fonction attendue')
plt.xlabel('Entree 1')
plt.subplot(1, 2, 2)
plt.imshow(prediction, extent=[0, 1, 0, 1])
plt.title('Cartopgrahie de la fonction predite')
plt.xlabel('Entree 1')
plt.ylabel('Entree 2')
plt.show()

model_json = model.to_json()
with open("model.json", "w") as json_file:
    json_file.write(model_json)
# serialize weights to HDF5
model.save_weights("model.h5")
print("Saved model to disk")
