from keras import layers
from keras import models
from keras import optimizers
from keras.regularizers import l2
from keras.preprocessing.image import ImageDataGenerator
import matplotlib.pyplot as plt

# layers
model = models.Sequential()
model.add(layers.Conv2D(16, (3, 3), kernel_regularizer=l2(0.003), activation='relu',
                        input_shape=(500, 500, 3)))
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Conv2D(32, (3, 3), kernel_regularizer=l2(0.003), activation='relu'))
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Conv2D(64, (3, 3), kernel_regularizer=l2(0.003), activation='relu'))
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Conv2D(64, (3, 3), kernel_regularizer=l2(0.003), activation='relu'))
model.add(layers.MaxPooling2D((2, 2)))
model.add(layers.Flatten())
model.add(layers.Dropout(0.5))
model.add(layers.Dense(256, kernel_regularizer=l2(0.003), activation='relu'))
model.add(layers.Dense(1, kernel_regularizer=l2(0.003), activation='sigmoid'))

# compile
model.compile(loss='binary_crossentropy',
              optimizer=optimizers.RMSprop(lr=1e-4),
              metrics=['acc'])

# training
train_datagen = ImageDataGenerator(
    rescale=1. / 255,
    rotation_range=40,
    width_shift_range=0.2,
    height_shift_range=0.2,
    zoom_range=0.2,
    horizontal_flip=True,
    vertical_flip=True,  # new
    fill_mode='nearest')

test_datagen = ImageDataGenerator(rescale=1. / 255)

train_generator = train_datagen.flow_from_directory(
    'C:/LanAnh/McGill/Robotics Rover - Science team/Test/train_dir',
    target_size=(500, 500),
    batch_size=20,
    class_mode='binary')

validation_generator = test_datagen.flow_from_directory(
    'C:/LanAnh/McGill/Robotics Rover - Science team/Test/validation_dir',
    target_size=(500, 500),
    batch_size=20,
    class_mode='binary')

test_generator = test_datagen.flow_from_directory(
    'C:/LanAnh/McGill/Robotics Rover - Science team/Test/test_dir',
    target_size=(500, 500),
    batch_size=20,
    class_mode='binary')

history = model.fit_generator(
    train_generator,
    steps_per_epoch=5,
    epochs=120,
    validation_data=validation_generator,
    validation_steps=5)

# evaluate the model
scores = model.evaluate(test_generator)
print('%s: %.2f%%' % (model.metrics_names[1], scores[1] * 100))

model.save('model_2.h5')
print('Saved model to disk')

# Displaying curves of loss and accuracy during training
acc = history.history['acc']
val_acc = history.history['val_acc']
loss = history.history['loss']
val_loss = history.history['val_loss']

epochs = range(1, len(acc) + 1)

plt.plot(epochs, acc, 'bo', label='Training acc')
plt.plot(epochs, val_acc, 'b', label='Validation acc')
plt.title('Training and validation accuracy')
plt.legend()

plt.figure()

plt.plot(epochs, loss, 'bo', label='Training loss')
plt.plot(epochs, val_loss, 'b', label='Validation loss')
plt.title('Training and validation loss')
plt.legend()

plt.show()
